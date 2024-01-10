namespace PhysicsSimulator

open System
open MathNet.Numerics.LinearAlgebra
open FSharpPlus
open FSharpPlus.Data
open Constants
open Utils

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

    type private SATAxisOrigin =
        | Faces1
        | Faces2
        | Edges

    type private PossibleCollisionData =
        { Normal: Vector3D
          Penetration: float }

    let private chooseSeparationAxis (candidatesMap: Map<SATAxisOrigin, PossibleCollisionData>) =
        candidatesMap |> Map.toList |> List.minBy (snd >> _.Penetration)

    let private detectBoxBoxCollision boxCollider1 body1 boxCollider2 body2 : CollisionData option =
        let getOrientedFaces colliderBox (rigidBody: RigidBody) =
            let getOrientedVertex =
                getOriented rigidBody.Variables.Orientation rigidBody.MassCenter.Variables.Position

            colliderBox
            |> Collider.getFaces
            |> Seq.map (fun face ->
                { Vertices = face.Vertices |> Seq.map getOrientedVertex
                  Face.Normal = face.Normal.Get * rigidBody.Variables.Orientation.Get |> ofVector })

        let tryGetCollisionDataForAxis vertices1 vertices2 (axis: Vector<float>) : PossibleCollisionData option =
            let withProjection (vertices: Vector<float> seq) =
                vertices |> Seq.map (fun v -> (v, v.DotProduct(axis)))

            let v1 = vertices1 |> withProjection
            let v2 = vertices2 |> withProjection

            let minMax =
                [ v1; v2 ]
                |> List.map (fun s ->
                    {| Min = s |> Seq.minBy snd
                       Max = s |> Seq.maxBy snd |})

            let min1, a = minMax[0].Min
            let max1, b = minMax[0].Max
            let min2, c = minMax[1].Min
            let max2, d = minMax[1].Max

            // a    c  b           d
            if a <= c && b >= c then
                let penetration = b - c // ??
                let normal = axis

                { Normal = normal |> ofVector
                  Penetration = penetration }
                // CollisionData.Create penetration normal [ max1 + normal * penetration |> ofVector ]
                |> Some
            // c     a     d  b
            elif c <= a && d >= a then
                let penetration = d - a // ??
                let normal = -axis // ???

                { Normal = normal |> ofVector
                  Penetration = penetration }
                //CollisionData.Create penetration normal [ min1 + normal * penetration |> ofVector ]
                |> Some
            else
                None

        let faces1 = getOrientedFaces boxCollider1 body1
        let faces2 = getOrientedFaces boxCollider2 body2

        let vertices1 = faces1 |> Seq.bind _.Vertices |> Seq.distinct |> Seq.map toVector
        let vertices2 = faces2 |> Seq.bind _.Vertices |> Seq.distinct |> Seq.map toVector

        let facesAxes = [ faces1; faces2 ] |> List.map (Seq.map (_.Normal >> toVector))

        let edgesAxes =
            facesAxes[1]
            |> Seq.apply (facesAxes[0] |> Seq.map crossProductV)
            |> Seq.map _.Normalize(2.0)

        let axes =
            [ Faces1, facesAxes[0]
              Faces2, facesAxes[1]
              // Edges, edgesAxes
              ]
            |> Map.ofList

        let bestAxes =
            axes
            |> Map.mapValues (
                Seq.map (tryGetCollisionDataForAxis vertices1 vertices2)
                >> Seq.sequence
                >> Option.map (Seq.minBy (_.Penetration))
            )

        let findIncidentFace (referenceFace: Face) (facesCandidates: Face seq) =
            facesCandidates |> Seq.minBy (_.Normal.Get.DotProduct(referenceFace.Normal.Get))

        if bestAxes |> Map.exists (fun _ axis -> axis.IsNone) then
            None
        else
            let separationAxis = bestAxes |> Map.mapValues Option.get |> chooseSeparationAxis

            let generateFaceContactPoints normal penetration referenceFaces otherFaces =
                let reference = Collider.findFaceByNormal normal referenceFaces
                let incidentFace = findIncidentFace reference otherFaces

                let adjacentFaces =
                    reference |> Collider.findAdjacentFaces referenceFaces |> Seq.toList

                let contactPoints =
                    incidentFace.Vertices
                    |> Seq.toList
                    |> GraphicsUtils.SutherlandHodgmanClipping(
                        adjacentFaces |> List.map (Face.toPlane >> Plane.invertNormal)
                    )
                    // clip against reference plane:
                    |> List.filter (fun vertex ->
                        vertex
                        |> GraphicsUtils.isPointInPlane (reference |> Face.toPlane |> Plane.invertNormal))
                    |> Seq.map (fun cp -> ContactPoint.Create penetration normal cp)

                { ContactPoints = contactPoints |> Seq.toList } |> Some

            match separationAxis with
            | Faces1,
              { Normal = normal
                Penetration = penetration } -> (faces1, faces2) ||> generateFaceContactPoints normal penetration
            | Faces2,
              { Normal = normal
                Penetration = penetration } -> (faces2, faces1) ||> generateFaceContactPoints normal penetration
            | Edges, { Normal = n; Penetration = f } -> failwith "edges"

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
            | Sphere sphere, Sphere sphere2 ->
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
            | Sphere sphere, Box box -> None
            | Box box, Sphere sphere -> None
            | Box box1, Box box2 -> detectBoxBoxCollision box1 body1 box2 body2
