namespace PhysicsSimulator.Collisions

open PhysicsSimulator
open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities

open MathNet.Numerics.LinearAlgebra
open FSharpPlus
open FSharpPlus.Data

type ContactPoint =
    { Normal: Vector3D
      Position: Vector3D
      Penetration: float }

    static member Create penetration normal position =
        if normal |> Vector3D.toVector |> Vector.norm |> equals 1.0 |> not then
            invalidArg "normal" "normal vector must me normalized"

        { Normal = normal
          Position = position
          Penetration = penetration }

type CollisionData =
    { ContactPoints: ContactPoint list }

    member this.WithInvertedNormals() =
        { ContactPoints =
            this.ContactPoints
            |> List.map (fun cp ->
                { cp with
                    Normal = cp.Normal |> Vector3D.apply (~-) }) }

module CollisionDetection =
    open Vector3D
    open SAT

    let private detectBoxBoxCollision
        (boxCollider1: Box)
        body1
        (boxCollider2: Box)
        (body2: RigidBody)
        : CollisionData option =

        let generateFaceContactPoints normal referenceFaces otherFaces =
            let findIncidentFace (referenceFace: Face) (facesCandidates: Face seq) =
                facesCandidates |> Seq.minBy (_.Normal.Get.DotProduct(referenceFace.Normal.Get))

            let referenceFace = referenceFaces |> Box.findFaceByNormal normal 
            let incidentFace = otherFaces |> findIncidentFace referenceFace 

            let adjacentFaces =
                referenceFace |> Box.findAdjacentFaces referenceFaces |> Seq.toList

            let clipAgainstReferencePlane vertices =
                vertices
                |> List.filter (GraphicsUtils.isPointInPlane (referenceFace |> Face.toPlane |> Plane.invertNormal))

            let calculatePenetration cpPosition =
                cpPosition
                - GraphicsUtils.getClosestPointToPoly cpPosition (referenceFace.Vertices |> Seq.toList)
                |> dotProduct normal

            let contactPoints =
                incidentFace.Vertices
                |> Seq.toList
                |> GraphicsUtils.SutherlandHodgmanClipping(
                    adjacentFaces |> List.map (Face.toPlane >> Plane.invertNormal)
                )
                |> clipAgainstReferencePlane
                |> Seq.map (fun cpPosition ->
                    ContactPoint.Create (cpPosition |> calculatePenetration) normal cpPosition)

            { ContactPoints = contactPoints |> Seq.toList } |> Some

        monad' {
            let! separationAxis =
                [ (boxCollider1, body1); (boxCollider2, body2) ]
                |> SetOf2.ofList
                |> tryFindSeparatingAxis

            let result =
                (match separationAxis with
                 | { Origin = origin
                     Reference = reference
                     Incident = incident
                     CollisionNormalFromReference = normal } as res when origin = Faces1 || origin = Faces2 ->

                     printfn $"Reference: {separationAxis.Origin}"
                     (reference.Faces, incident.Faces) ||> generateFaceContactPoints normal)

            return! result
        }

    open SetOf2

    /// Collision Normal vector points from the first to the second object in pair
    let areColliding (pair: SimulatorObject SetOf2) : CollisionData option =

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

                    let contactPoint1 = firstPos + -normal * sphere.Radius
                    let contactPoint2 = secondPos + normal * sphere2.Radius

                    let contactPoint = (contactPoint1 + (contactPoint2 - contactPoint1) / 2.0)

                    { ContactPoints = [ (ContactPoint.Create 0.0 normal contactPoint) ] } |> Some
            | Collider.Sphere sphere, Collider.Box box -> None
            | Collider.Box box, Collider.Sphere sphere -> None
            | Collider.Box box1, Collider.Box box2 -> detectBoxBoxCollision box1 body1 box2 body2
