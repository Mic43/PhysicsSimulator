namespace PhysicsSimulator

open FSharpPlus
open MathNet.Numerics.LinearAlgebra
open FSharp.Data.UnitSystems.SI.UnitSymbols
open MathNet.Numerics.LinearAlgebra
type Matrix3 =
    private
    | Value of Matrix<float>
    member this.Get() =
        match this with
        | Value v -> v

type Vector3D =
    private
    | Value of Vector<float>
    member this.Get() =
        match this with
        | Value v -> v

    member this.X =
        match this with
        | Value v -> v.At(0)

    member this.Y =
        match this with
        | Value v -> v.At(1)

    member this.Z =
        match this with
        | Value v -> v.At(2)

module Vector3D =
    let create x y z = vector [ x; y; z ] |> Vector3D.Value
    let zero = create 0.0 0.0 0.0
    let (|Vector3D|) (Value value) = value
    let fromVector vec = vec |> Value
    let toVector (vec3d : Vector3D) = vec3d.Get()
    
    let crossProduct (first: Vector3D) (second: Vector3D) =
        let x =
            (first.Y * second.Z) - (first.Z * second.Y)

        let y =
            (first.Z * second.X) - (first.X * second.Z)

        let z =
            (first.X * second.Y) - (first.Y * second.X)

        (x, y, z) |||> create


module Matrix3 =
    let (|Matrix3|) (Value value) = value
    let fromMatrix matrix = matrix |> Matrix3.Value

module RotationMatrix3D =
    open Vector3D

    let zero =
        Matrix<float>.Build.DenseIdentity 3
        |> Matrix3.Value

    let fromAxisAndAngle (Vector3D axis) angle =
        if axis.L2Norm() <> 1.0 then
            invalidArg "axis" "axis must be unit vector"

        let ux = axis.Item(0)
        let uy = axis.Item(1)
        let uz = axis.Item(2)

        let cosA = angle |> cos
        let cosInv = 1.0 - cosA
        let sinA = angle |> sin


        matrix [ [ cos angle + ux * ux * cosInv
                   ux * uy * cosInv - uz * sinA
                   ux * uz * cosInv + uy * sinA ]
                 [ uy * ux * cosInv + uz * sinA
                   cosA + uy * uy * cosInv
                   uy * uz * cosInv - ux * sinA ]
                 [ uz * ux * cosInv - uy * sinA
                   uz * uy * cosA + ux * sinA
                   cosA + uz * uz * cosInv ] ]
        |> Matrix3.Value
