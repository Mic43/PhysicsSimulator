namespace PhysicsSimulator.Collisions

open FSharpPlus.Data
open PhysicsSimulator
open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities
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

module internal CollisionDetection =
    open Vector3D
    open SAT

    let private detectBoxBoxCollision (boxCollider1, body1) (boxCollider2, body2) =
        let generateFaceContactPoints normal referenceFaces otherFaces : Reader<StepConfiguration, CollisionData Option> =
            monad {
                let! (config: StepConfiguration) = ask
                let epsilon = config.Epsilon

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

                if contactPoints |> Seq.isEmpty then
                    None
                else
                    return { ContactPoints = contactPoints |> Seq.toList } |> Some
            }

        let generateEdgeContactPoints collisionNormalFromFirstBox penetration boxes edgeAxes =
            let getSupportingEdge epsilon collisionNormal orientedBox edgeAxis =

                let edgesCandidates =
                    orientedBox
                    |> OrientedBox.getEdges
                    |> List.filter (fun edge ->
                        (edge |> SetOf2.snd) - (edge |> SetOf2.fst)
                        |> areParallel epsilon (edgeAxis |> NormalVector.toVector3D))

                edgesCandidates
                |> List.maxBy (
                    SetOf2.map (dotProduct (collisionNormal |> NormalVector.toVector3D))
                    >> SetOf2.max
                )

            ///get closest point between two edges
            let getContactPoint (edge1: Vector3D SetOf2) (edge2: Vector3D SetOf2) =
                let DA = (edge1 |> SetOf2.snd) - (edge1 |> SetOf2.fst)
                let DB = (edge2 |> SetOf2.snd) - (edge2 |> SetOf2.fst)

                let r = (edge1 |> SetOf2.fst) - (edge2 |> SetOf2.fst)
                let a = DA |> dotProduct DA
                let e = DB |> dotProduct DB
                let f = DB |> dotProduct r
                let c = DA |> dotProduct r

                let b = DA |> dotProduct DB
                let denom = a * e - b * b

                let TA = (b * f - c * e) / denom
                let TB = (b * TA + f) / e

                let CA = (edge1 |> SetOf2.fst) + DA * TA
                let CB = (edge2 |> SetOf2.fst) + DB * TB

                CA * 0.5 + CB * 0.5

            monad {
                let! config = ask

                let collidingEdges =
                    (boxes, edgeAxes, [ collisionNormalFromFirstBox; -collisionNormalFromFirstBox ] |> SetOf2.ofList)
                    |||> SetOf2.zip3
                    |> SetOf2.map (fun (box, edgeAxis, normal) ->
                        edgeAxis |> getSupportingEdge config.Epsilon normal box)

                let cp =
                    getContactPoint (collidingEdges |> SetOf2.fst) (collidingEdges |> SetOf2.snd)

                let contactPoints =
                    { Normal = collisionNormalFromFirstBox
                      Position = cp
                      Penetration = -penetration }
                    |> List.singleton

                { ContactPoints = contactPoints } |> Some
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

                generateEdgeContactPoints normal penetration ([ refBox; incidentBox ] |> SetOf2.ofList) axes.EdgesAxes

    let private detectedSphereBoxCollision (sphere, bodySphere) (box, bodyBox) =
        monad {
            let! conf = ask

            SphereBox.tryFindSeparatingAxis (sphere, bodySphere) (box, bodyBox)
            |> Option.map (fun result ->
                let startP = result.SphereCenter
                let endP = result.SphereCenter - result.CollisionNormalFromBox.Get * sphere.Radius

                let position =
                    (startP, endP)
                    ||> GraphicsUtils.tryGetPlaneEdgeIntersection conf.Epsilon (result.CollidedFace |> Face.toPlane)
                    |> Option.get

                { ContactPoints =
                    (result.Penetration, result.CollisionNormalFromBox, position)
                    |||> ContactPoint.Create
                    |> List.singleton })
        }

    let private detectSphereSphereCollision (sphere, body1: RigidBody) (sphere2, body2: RigidBody) =
        let firstPos = body1.MassCenterPosition
        let secondPos = body2.MassCenterPosition
        let normal = firstPos - secondPos
        let dist = normal |> l2Norm

        if dist > sphere.Radius + sphere2.Radius then
            None
        else
            let normal = normal |> normalized

            let contactPoint1 = firstPos + -normal.Get * sphere.Radius
            let contactPoint2 = secondPos + normal.Get * sphere2.Radius

            let contactPoint = (contactPoint1 + (contactPoint2 - contactPoint1) / 2.0)
            let penetration = (contactPoint1 - contactPoint2) |> l2Norm

            { ContactPoints = [ (ContactPoint.Create penetration normal contactPoint) ] }
            |> Some
        |> Reader.Return

    /// Collision Normal vector points from the first to the second object in pair
    let areColliding (pair: SimulatorObject SetOf2) : Reader<StepConfiguration, CollisionData Option> =      
        let first = pair |> SetOf2.fst
        let second = pair |> SetOf2.snd

        match (first.PhysicalObject, second.PhysicalObject) with
        | RigidBody body1, RigidBody body2 ->
            match (first.Collider, second.Collider) with
            | Collider.Sphere sp1, Collider.Sphere sp2 -> detectSphereSphereCollision (sp1, body1) (sp2, body2)
            | Collider.Box box1, Collider.Box box2 -> detectBoxBoxCollision (box1, body1) (box2, body2)
            | Sphere sphere, Box box ->
                detectedSphereBoxCollision (sphere, body1) (box, body2)
                |> Reader.map (Option.map (_.WithInvertedNormals()))
            | Box box, Sphere sphere -> detectedSphereBoxCollision (sphere, body2) (box, body1)
