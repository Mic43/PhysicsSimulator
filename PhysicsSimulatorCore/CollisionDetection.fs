namespace PhysicsSimulator

open MathNet.Numerics.LinearAlgebra
open FSharpPlus
open FSharpPlus.Data

type CollisionData =
    private
        { Normal: Vector3D
          ContactPoint: Vector3D
          Penetration: float }

    static member Create penetration normal contactPoint =
        let eps = 0.00001

        if (normal |> Vector3D.toVector |> Vector.norm) - 1.0 > eps then
            invalidArg "normal" "normal vector must me normalized"

        { Normal = normal
          ContactPoint = contactPoint
          Penetration = penetration }

module CollisionDetection =
    open Vector3D

    let private getVertices colliderBox (rigidBody: RigidBody) =
        Collider.getVertices colliderBox
        |> Seq.map (fun v -> v.Get() * rigidBody.Variables.Orientation.Get())
        |> Seq.map (fun v -> v + rigidBody.MassCenter.Variables.Position.Get())

    let areColliding first second : CollisionData option =
        let firstPos = first.PhysicalObject.MassCenterPosition()
        let secondPos = second.PhysicalObject.MassCenterPosition()

        match (first.PhysicalObject, second.PhysicalObject) with
        | RigidBody body1, RigidBody body2 ->
            match (first.Collider, second.Collider) with
            | Sphere sphere, Sphere sphere2 ->
                let normal = firstPos.Get() - secondPos.Get()
                let dist = normal.L2Norm()

                if dist > sphere.Radius + sphere2.Radius then
                    None
                else
                    let normal = normal.Normalize(2.0)

                    let contactPoint1 = firstPos.Get() + -normal * sphere.Radius
                    let contactPoint2 = secondPos.Get() + normal * sphere2.Radius

                    let contactPoint =
                        (contactPoint1 + (contactPoint2 - contactPoint1) / 2.0 |> Vector3D.ofVector)

                    CollisionData.Create 0.0 normal contactPoint |> Some
            | Sphere sphere, Box box ->  None
            | Box box, Sphere sphere -> None
            | Box box1, Box box2 ->

                let getCollisionDataForAxis
                    (vertices1: Vector<float> seq)
                    (vertices2: Vector<float> seq)
                    (axis: Vector<float>)
                    =
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

                    if a <= c && b >= c then
                        let penetration = b - c // ??
                        let normal = axis

                        CollisionData.Create penetration normal (max1 + normal * penetration |> ofVector)
                        |> Some
                    elif c <= a && d >= a then
                        let penetration = d - a // ??
                        let normal = -axis

                        CollisionData.Create penetration normal (min1 + normal * penetration |> ofVector)
                        |> Some
                    else
                        None

                let facesAxes =
                    [ body1; body2 ]
                    |> List.map _.Variables.Orientation
                    |> List.map _.Get().EnumerateColumns()

                let edgesAxes =
                    facesAxes[1]
                    |> Seq.apply (facesAxes[0] |> Seq.map crossProductV)
                    |> Seq.map _.Normalize(2.0)

                let axes =
                    seq {
                        facesAxes |> Seq.bind id
                        edgesAxes
                    }
                    >>= id
                    |> Seq.filter (fun v -> v <> vector [ 0.0; 0.0; 0.0 ])

                let vertices1 = getVertices box1 body1
                let vertices2 = getVertices box2 body2

                axes
                |> Seq.map (fun axis -> axis |> getCollisionDataForAxis vertices1 vertices2)
                |> Seq.sequence
                |> Option.map (Seq.minBy (_.Penetration))
