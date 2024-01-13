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
        
type Sphere =
    { Radius: float }

    member this.CreateRotationalInertia mass =
        let I = 2.0 * mass * this.Radius * this.Radius / 5.0
        Matrix<float>.Build.Diagonal(3, 3, I) |> Matrix3.ofMatrix

type Box =
    { XSize: float
      YSize: float
      ZSize: float }

    member this.CreateRotationalInertia mass =
        let m = mass / 12.0
        let a2 = this.XSize * this.XSize
        let b2 = this.YSize * this.YSize
        let c2 = this.ZSize * this.ZSize

        Matrix<float>.Build.Diagonal [| m * (b2 + c2); m * (a2 + c2); m * (a2 + b2) |]
        |> Matrix3.ofMatrix

module Box = 
    let create x y z =
        { XSize = x
          YSize = y
          ZSize = z }
    let createCube size = (size,size,size) |||> create