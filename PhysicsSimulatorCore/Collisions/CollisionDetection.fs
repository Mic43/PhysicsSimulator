namespace PhysicsSimulator.Collisions

open FSharpPlus.Data
open PhysicsSimulator
open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities

open MathNet.Numerics.LinearAlgebra
open FSharpPlus

type ContactPoint =
    { Normal: NormalVector
      Position: Vector3D
      Penetration: float }

    static member Create penetration normal position =
        { Normal = normal
          Position = position
          Penetration = penetration }

type CollisionData =
    { ContactPoints: ContactPoint list }

    member this.WithInvertedNormals() =
        { ContactPoints = this.ContactPoints |> List.map (fun cp -> { cp with Normal = -cp.Normal }) }

module CollisionDetection =
    open Vector3D
    open SAT

    let private detectBoxBoxCollision
        (boxCollider1: Box)
        body1
        (boxCollider2: Box)
        body2
        : Reader<Configuration, CollisionData Option> =

        let generateFaceContactPoints normal referenceFaces otherFaces : Reader<Configuration, CollisionData Option> =
            monad {
                let! (config: Configuration) = ask
                let epsilon = config.epsilon

                let findIncidentFace (referenceFace: Face) (facesCandidates: Face seq) =
                    facesCandidates
                    |> Seq.minBy (fun f -> f.Normal.Get |> dotProduct referenceFace.Normal.Get)

                let referenceFace = referenceFaces |> Box.findFaceByNormal normal
                let incidentFace = otherFaces |> findIncidentFace referenceFace

                let calculatePenetration cpPosition =
                    cpPosition
                    - GraphicsUtils.getClosestPointToPoly cpPosition (referenceFace.Vertices |> Seq.toList)
                    |> dotProduct normal.Get

                let clipped incidentFaceVertices =
                    let adjacentFaces =
                        referenceFace |> Box.findAdjacentFaces epsilon referenceFaces |> Seq.toList

                    // keep only vertices below reference face
                    let clipAgainstReferencePlane vertices =
                        let clipPlane = referenceFace |> Face.toPlane |> Plane.inverted
                        vertices |> List.filter (GraphicsUtils.isPointInPlane clipPlane)

                    incidentFaceVertices
                    |> Seq.toList
                    |> GraphicsUtils.SutherlandHodgmanClipping
                        epsilon
                        (adjacentFaces |> List.map (Face.toPlane >> Plane.inverted))
                    |> clipAgainstReferencePlane

                let contactPoints =
                    incidentFace.Vertices
                    |> clipped
                    |> Seq.map (fun cpPosition ->
                        ContactPoint.Create (cpPosition |> calculatePenetration) normal cpPosition)

                return { ContactPoints = contactPoints |> Seq.toList } |> Some
            }

        let generateEdgeContactPoints collisionNormal penetration boxes edgeAxes =
            let getSupportingEdge epsilon collisionNormal orientedBox axis =

                let edgesCandidates =
                    orientedBox
                    |> OrientedBox.getEdges
                    |> List.filter (fun edge ->
                        (edge |> SetOf2.fst) - (edge |> SetOf2.snd)
                        |> areParallel epsilon (axis |> NormalVector.toVector3D))

                edgesCandidates
                |> List.maxBy (
                    SetOf2.map (dotProduct (collisionNormal |> NormalVector.toVector3D))
                    >> SetOf2.max
                )

            let getContactPoint (edge1: Vector3D SetOf2) (edge2: Vector3D SetOf2) =
                let pOne = edge1 |> SetOf2.fst
                let pTwo = edge2 |> SetOf2.fst

                let dOne = (edge1 |> SetOf2.snd) - pOne
                let dTwo = (edge2 |> SetOf2.snd) - pTwo

                let smOne = pOne.X * pOne.X + pOne.Y * pOne.Y + pOne.Z * pOne.Z
                let smTwo = pTwo.X * pTwo.X + pTwo.Y * pTwo.Y + pTwo.Z * pTwo.Z
                let dpOneTwo = dTwo |> dotProduct dOne

                let toSt = pOne - pTwo
                let dpStaOne = dOne |> dotProduct toSt
                let dpStaTwo = dTwo |> dotProduct toSt

                let denom = smOne * smTwo - dpOneTwo * dpOneTwo
                let mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom
                let mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom

                let cOne = pOne + dOne * mua
                let cTwo = pTwo + dTwo * mub

                cOne * 0.5 + cTwo * 0.5

            monad {
                let! config = ask

                let edges =
                    (boxes, edgeAxes)
                    ||> SetOf2.zip
                    |> SetOf2.map (fun (box, axis) -> axis |> getSupportingEdge config.epsilon collisionNormal box)

                let cp = getContactPoint (edges |> SetOf2.fst) (edges |> SetOf2.snd)

                { ContactPoints =
                    { Normal = collisionNormal
                      Position = cp
                      Penetration = penetration }
                    |> List.singleton }
                |> Some
            }

        let separationAxis =
            [ (boxCollider1, body1); (boxCollider2, body2) ]
            |> SetOf2.ofList
            |> tryFindSeparatingAxis

        match separationAxis with
        | None -> None |> Reader.Return
        | Some separationAxis ->
            match separationAxis with
            | { Origin = Face1
                Reference = reference
                Incident = incident
                CollisionNormalFromReference = normal } ->

                (reference.Faces, incident.Faces) ||> generateFaceContactPoints normal
            | { Origin = Face2
                Reference = reference
                Incident = incident
                CollisionNormalFromReference = normal } ->

                (reference.Faces, incident.Faces)
                ||> generateFaceContactPoints normal
                // we need to invert normals so it points from body1 to body2 regardless of the separation axis
                |> Reader.map (Option.map (_.WithInvertedNormals()))
            | { Origin = Edge(axes)
                Reference = refBox
                Incident = incidentBox
                Penetration = penetration
                CollisionNormalFromReference = normal } ->

                // Option.None |> Reader.Return
                generateEdgeContactPoints normal penetration ([ refBox; incidentBox ] |> SetOf2.ofList) axes.EdgesAxes

    open SetOf2

    /// Collision Normal vector points from the first to the second object in pair
    let areColliding (pair: SimulatorObject SetOf2) : Reader<Configuration, CollisionData Option> =

        let objects = pair |> map _.PhysicalObject
        let positions = objects |> map _.MassCenterPosition()

        let first = pair |> fst
        let second = pair |> snd

        match (first.PhysicalObject, second.PhysicalObject) with
        | RigidBody body1, RigidBody body2 ->
            match (first.Collider, second.Collider) with
            | Collider.Sphere sphere, Collider.Sphere sphere2 ->
                let firstPos = positions |> fst
                let secondPos = positions |> snd
                let normal = firstPos - secondPos
                let dist = normal |> l2Norm

                if dist > sphere.Radius + sphere2.Radius then
                    None
                else
                    let normal = normal |> normalized

                    let contactPoint1 = firstPos + -normal.Get * sphere.Radius
                    let contactPoint2 = secondPos + normal.Get * sphere2.Radius

                    let contactPoint = (contactPoint1 + (contactPoint2 - contactPoint1) / 2.0)

                    { ContactPoints = [ (ContactPoint.Create 0.0 normal contactPoint) ] } |> Some
                |> Reader.Return
            | Collider.Box box1, Collider.Box box2 -> detectBoxBoxCollision box1 body1 box2 body2
