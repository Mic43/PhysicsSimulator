namespace PhysicsSimulator.Entities

open MathNet.Numerics.LinearAlgebra
open PhysicsSimulator.Utilities

type Acceleration = Vector3D

type Torque = Vector3D

type RotationalInertiaInverse = Matrix3

type Mass =
    | Infinite
    | Value of float

    member this.GetValue() =
        match this with
        | Infinite -> infiniteMass
        | Value v -> v

type Face =
    { Vertices: Vector3D seq
      Normal: Vector3D }

module Face =
    let toPlane face =
        face.Normal
        |> Plane.create ((face.Vertices |> Seq.head).Get.DotProduct(face.Normal.Get))

//face.Normal |> Vector3D.crossProduct targetFace.Normal |> abs > epsilon)
