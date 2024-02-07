namespace PhysicsSimulator.Utilities

open MathNet.Numerics.LinearAlgebra

[<RequireQualifiedAccess>]
type Matrix3 =
    private
    | Value of Matrix<float>

    member this.Get =
        match this with
        | Value v -> v

module Matrix3 =    
    let (|Matrix3|) (Matrix3.Value value) = value

    let ofMatrix (matrix: Matrix<float>) =
        if matrix.RowCount <> 3 || matrix.ColumnCount <> 3 then
            invalidArg "matrix" "matrix must be of size 3"

        matrix |> Matrix3.Value

    let zero = Matrix<float>.Build.Dense(3, 3) |> ofMatrix

    let hatOperator (vector: Vector3D) =
        matrix
            [ [ 0.0; -vector.Z; vector.Y ]
              [ vector.Z; 0.0; -vector.X ]
              [ -vector.Y; vector.X; 0.0 ] ]
        |> Matrix3.Value

    let orthonormalize (matrix: Matrix3) =
        let matrix = matrix.Get
      
        let deltaCol (c1: Vector<float>) c2 =
            c1.DotProduct(c2) / c1.DotProduct(c1) * c1

        let col0 = matrix.Column(0).Normalize(2.0)
        let mutable col1 = matrix.Column(1)
        let mutable col2 = matrix.Column(2)

        col1 <- col1 - deltaCol col0 col1
        col1 <- col1 - deltaCol col0 col1
        col1 <- col1.Normalize(2.0)

        col2 <- col2 - deltaCol col0 col2
        col2 <- col2 - deltaCol col1 col2
        col2 <- col2 - deltaCol col0 col2
        col2 <- col2 - deltaCol col1 col2
        col2 <- col2.Normalize(2.0)

        Matrix.Build.DenseOfColumns([ col0; col1; col2 ]) |> ofMatrix

module RotationMatrix3D =
    // open Vector3D
    let zero = Matrix<float>.Build.DenseIdentity 3 |> Matrix3.Value

    let fromAxisAndAngle (NormalVector.Value axis) angle =
        let axis = axis.Get
        
        let ux = axis.Item(0)
        let uy = axis.Item(1)
        let uz = axis.Item(2)

        let cosA = angle |> cos
        let cosInv = 1.0 - cosA
        let sinA = angle |> sin

        matrix
            [ [ cosA + ux * ux * cosInv
                ux * uy * cosInv - uz * sinA
                ux * uz * cosInv + uy * sinA ]
              [ uy * ux * cosInv + uz * sinA
                cosA + uy * uy * cosInv
                uy * uz * cosInv - ux * sinA ]
              [ uz * ux * cosInv - uy * sinA
                uz * uy * cosInv + ux * sinA
                cosA + uz * uz * cosInv ] ]
        |> Matrix3.Value

    let fromYawPitchRoll yaw pitch roll =
        let cosA = yaw |> cos
        let sinA = yaw |> sin

        let cosB = pitch |> cos
        let sinB = pitch |> sin

        let cosG = roll |> cos
        let sinG = roll |> sin

        matrix
            [ [ cosB * cosG
                sinA * sinB * cosG - cosA * sinG
                cosA * sinB * cosG + sinA * sinG ]
              [ cosB * sinG
                cosA * sinB * sinG + cosA * cosG
                cosA * sinB * sinG - sinA * cosG ]
              [ -sinB; sinA * cosB; cosA * cosB ] ]
        |> Matrix3.Value
