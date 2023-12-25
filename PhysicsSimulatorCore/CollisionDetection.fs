namespace PhysicsSimulator

open MathNet.Numerics.LinearAlgebra
open FSharpPlus
open FSharpPlus.Data
open Constants

type CollisionData =
    { Normal: Vector3D
      ContactPoints: Vector3D seq
      Penetration: float }

    static member Create penetration normal contactPoint =

        if (normal |> Vector3D.toVector |> Vector.norm) - 1.0 > epsilon then
            invalidArg "normal" "normal vector must me normalized"

        { Normal = normal
          ContactPoints = contactPoint
          Penetration = penetration }

module CollisionDetection =
    open Vector3D

    type private SATAxisOrigin =
        | Faces1
        | Faces2
        | Edges

    type private PossibleCollisionData =
        { Normal: Vector<float>
          Penetration: float }

    let private chooseSeparationAxis (candidatesMap: Map<SATAxisOrigin, PossibleCollisionData>) =
        candidatesMap |> Map.toList |> List.minBy (snd >> _.Penetration)

    let private detectBoxBoxCollision boxCollider1 body1 boxCollider2 body2 : CollisionData option =
        // let getVertices colliderBox (rigidBody: RigidBody) =
        //     Collider.getVertices colliderBox
        //     |> Seq.map (fun v -> v.Get * rigidBody.Variables.Orientation.Get)
        //     |> Seq.map (fun v -> v + rigidBody.MassCenter.Variables.Position.Get)

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

                { Normal = normal
                  Penetration = penetration }
                // CollisionData.Create penetration normal [ max1 + normal * penetration |> ofVector ]
                |> Some
            // c     a     d  b
            elif c <= a && d >= a then
                let penetration = d - a // ??
                let normal = -axis

                { Normal = normal
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
        //  |> Map.filter (fun _ value -> value |> Option.isSome)
        //            |> Map.mapValues Option.get

        let findIncidentFace (referenceFace: Face) (facesCandidates: Face seq) =
            facesCandidates |> Seq.minBy (_.Normal.Get.DotProduct(referenceFace.Normal.Get))

        if bestAxes |> Map.exists (fun _ axis -> axis.IsNone) then
            None
        else
            let separationAxis = bestAxes |> Map.mapValues Option.get |> chooseSeparationAxis

            match separationAxis with
            | Faces1,
              { Normal = normal
                Penetration = penetration } ->
                let reference = Collider.findFaceByNormal normal faces1
                let incidentFace = findIncidentFace reference faces2
                let adjacentFaces = reference |> Collider.findAdjacentFaces faces1 |> Seq.toList

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


                { CollisionData.Normal = normal
                  ContactPoints = contactPoints
                  Penetration = penetration }
                |> Some
            | Faces2, { Normal = n; Penetration = f } -> failwith "todo"
            | Edges, { Normal = n; Penetration = f } -> failwith "edges"

    let areColliding first second : CollisionData option =
        let firstPos = first.PhysicalObject.MassCenterPosition()
        let secondPos = second.PhysicalObject.MassCenterPosition()

        match (first.PhysicalObject, second.PhysicalObject) with
        | RigidBody body1, RigidBody body2 ->
            match (first.Collider, second.Collider) with
            | Sphere sphere, Sphere sphere2 ->
                let normal = firstPos.Get - secondPos.Get
                let dist = normal.L2Norm()

                if dist > sphere.Radius + sphere2.Radius then
                    None
                else
                    let normal = normal.Normalize(2.0)

                    let contactPoint1 = firstPos.Get + -normal * sphere.Radius
                    let contactPoint2 = secondPos.Get + normal * sphere2.Radius

                    let contactPoint =
                        (contactPoint1 + (contactPoint2 - contactPoint1) / 2.0 |> ofVector)

                    CollisionData.Create 0.0 normal [ contactPoint ] |> Some
            | Sphere sphere, Box box -> None
            | Box box, Sphere sphere -> None
            | Box box1, Box box2 -> detectBoxBoxCollision box1 body1 box2 body2
