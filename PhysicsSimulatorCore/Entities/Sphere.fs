namespace PhysicsSimulator.Entities

open MathNet.Numerics.LinearAlgebra
open PhysicsSimulator.Utilities


type Sphere =
    { Radius: float }

    member this.CreateRotationalInertia mass =
        let I = 2.0 * mass * this.Radius * this.Radius / 5.0
        Matrix<float>.Build.Diagonal(3, 3, I) |> Matrix3.ofMatrix

module Sphere = 
    let create radius =
        do throwIfNegative radius (nameof (radius))

        { Radius = radius } 