namespace PhysicsSimulator.Utilities

open MathNet.Numerics.LinearAlgebra

[<RequireQualifiedAccess>]
type Matrix3 =
    private
    | Value of Matrix<float>

    member this.Get =
        match this with
        | Value v -> v

    static member (*)(scalar: float, m: Matrix3) = scalar * m.Get |> Value
    static member (*)(m1: Matrix3, m2: Matrix3) = m1.Get * m2.Get |> Value
    static member (*)(v: Vector3D, m: Matrix3) = v.Get * m.Get |> Vector3D.ofVector
    static member (*)(m: Matrix3, v: Vector3D) = m.Get * v.Get |> Vector3D.ofVector
    static member (+)(m1: Matrix3, m2: Matrix3) = m1.Get + m2.Get |> Value
    static member (-)(m1: Matrix3, m2: Matrix3) = m1.Get - m2.Get |> Value

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

    let transposed (matrix: Matrix3) = matrix.Get.Transpose() |> Matrix3.Value
    let inverted (matrix: Matrix3) = matrix.Get.Inverse() |> Matrix3.Value
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

    /// Inverse of `fromYawPitchRoll` (columns = body X, Y, Z in world space).
    let yawPitchRollFromMatrix (Matrix3.Value m) =
        let pitch = -asin (max -1.0 (min 1.0 m.[2, 0]))
        let cosP = cos pitch

        let roll =
            if abs cosP < 1e-6 then
                0.0
            else
                atan2 m.[1, 0] m.[0, 0]

        let yaw =
            if abs cosP < 1e-6 then
                atan2 (-m.[0, 1]) m.[1, 1]
            else
                atan2 m.[2, 1] m.[2, 2]

        yaw, pitch, roll

    /// Body X = tangent, Y = binormal, Z = normal (matches `fromYawPitchRoll`).
    let yawPitchRollFromFrame (tangent: Vector3D) (binormal: Vector3D) (normal: Vector3D) =
        matrix
            [ [ tangent.X; binormal.X; normal.X ]
              [ tangent.Y; binormal.Y; normal.Y ]
              [ tangent.Z; binormal.Z; normal.Z ] ]
        |> Matrix3.ofMatrix
        |> Matrix3.orthonormalize
        |> yawPitchRollFromMatrix
