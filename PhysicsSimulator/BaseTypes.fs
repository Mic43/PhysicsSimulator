namespace PhysicsSimulator

open FSharpPlus
open MathNet.Numerics.LinearAlgebra


type RotationMatrix3D =
    private
    | Value of Matrix<float>
    member this.Get() =
        function
        | Value v -> v

type Vector3D =
    private
    | Value of Vector<float>
    member this.Get() =
        function
        | Value v -> v


module Vector3D =
    let create x y z = vector [ x; y; z ] |> Vector3D.Value

module RotationMatrix3D =
    let zero =
        Matrix<float>.Build.DenseIdentity 3

    let (|Vector3D|) (Value value) = value
    let (|RotationMatrix3D|) (Value value) = value


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
        |> RotationMatrix3D.Value
