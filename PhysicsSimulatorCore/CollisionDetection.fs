namespace PhysicsSimulator

open MathNet.Numerics.LinearAlgebra

type CollisionData =
    private
        { Normal: Vector3D
          ContactPoint: Vector3D }

    static member Create normal contactPoint =
        let eps = 0.00001

        if (normal |> Vector3D.toVector |> Vector.norm) - 1.0 > eps then
            invalidArg "normal" "normal vector must me normalized"

        { Normal = normal
          ContactPoint = contactPoint }


module CollisionDetection =

    let areColliding first second : CollisionData option =

        match (first.Collider, second.Collider) with
        | Sphere sphere, Sphere sphere2 ->
            let firstPos = first.PhysicalObject.MassCenterPosition()
            let secondPos = second.PhysicalObject.MassCenterPosition()

            let normal = secondPos.Get() - firstPos.Get()
            let dist = normal.L2Norm()

            if dist > sphere.Radius + sphere2.Radius then
                None
            else
                let normal = normal.Normalize(2.0)

                let contactPoint1 = firstPos.Get() + -normal * sphere.Radius
                let contactPoint2 = secondPos.Get() + normal * sphere2.Radius

                CollisionData.Create normal (contactPoint1 + (contactPoint2 - contactPoint1) / 2.0 |> Vector3D.ofVector)
                |> Some


        | Sphere sphere, Box box -> failwith "todo"
        | Box box, Sphere sphere -> failwith "todo"
        | Box box, Box box1 -> failwith "todo"
