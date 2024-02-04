namespace PhysicsSimulator.Entities

open MathNet.Numerics.LinearAlgebra
open PhysicsSimulator.Utilities

type Acceleration = Vector3D

type Torque = Vector3D

type RotationalInertiaInverse = Matrix3

[<RequireQualifiedAccess>]
type Mass =
    | Infinite
    | Value of float

    member this.GetValue =
        match this with
        | Infinite -> infinity
        | Value v -> v

type Face =
    { Vertices: Vector3D seq
      Normal: NormalVector }

module Face =
    let toPlane face =
        face.Normal
        |> Plane.create  (face.Vertices |> Seq.head |> Vector3D.dotProduct face.Normal.Get)
