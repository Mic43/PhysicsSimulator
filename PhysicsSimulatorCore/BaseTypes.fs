namespace PhysicsSimulator

open FSharpPlus
open MathNet.Numerics.LinearAlgebra
open FSharp.Data.UnitSystems.SI.UnitSymbols
open MathNet.Numerics.LinearAlgebra

// type SetOf2<'T when 'T: comparison> =
//     private Content of Set<'T>
//
//
//     member

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

    member this.HatOperator() =
        matrix [ [ 0.0; -this.Z; this.Y ]; [ this.Z; 0.0; -this.X ]; [ -this.Y; this.X; 0.0 ] ]
        |> Matrix3.Value

    static member op_Implicit(v: Vector<float>) = v |> Vector3D.Value

module Vector3D =
    let create x y z = vector [ x; y; z ] |> Vector3D.Value
    let zero = create 0.0 0.0 0.0
    let (|Vector3D|) (Value value) = value
    let ofVector vec = vec |> Value
    let toVector (vec3d: Vector3D) = vec3d.Get()

    let apply (f: Vector<float> -> Vector<float>) (v: Vector3D) = v |> toVector |> f |> ofVector

    let apply2 (f: Vector<float> -> Vector<float> -> Vector<float>) (v: Vector3D) (v2: Vector3D) =
        (v |> toVector, v2 |> toVector) ||> f |> ofVector

    let crossProduct (first: Vector3D) (second: Vector3D) =
        let x = (first.Y * second.Z) - (first.Z * second.Y)
        let y = (first.Z * second.X) - (first.X * second.Z)
        let z = (first.X * second.Y) - (first.Y * second.X)

        (x, y, z) |||> create

    let crossProductV (first) (second) =
        second |> ofVector |> crossProduct (first |> ofVector) |> toVector

module Matrix3 =
    open Vector3D
    let (|Matrix3|) (Value value) = value

    let fromMatrix (matrix: Matrix<float>) =
        if matrix.RowCount <> 3 || matrix.ColumnCount <> 3 then
            invalidArg "matrix" "matrix must be of size 3"

        matrix |> Matrix3.Value

    let orthonormalize (matrix: Matrix3) =

        let matrix = matrix.Get()
        // printfn "Before normalization %A" matrix

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

        Matrix.Build.DenseOfColumns([ col0; col1; col2 ]) |> fromMatrix

    let reorthogonalise2 (m: Matrix3) =
        let x = m.Get().Row(0)
        let y = m.Get().Row(1)
        let z = m.Get().Row(2)

        let xORt = (crossProductV y z)
        let yOrt = (crossProductV z x)
        let zORt = z

        Matrix<float>.Build
            .DenseOfRowVectors(xORt.Normalize(2.0), yOrt.Normalize(2.0), zORt.Normalize(2.0))
        |> fromMatrix

    let reorthogonalise (m: Matrix3) =
        let x = m.Get().Row(0)
        let y = m.Get().Row(1)

        let error = x.DotProduct(y)

        let xORt = x - (error / 2.0) * y
        let yOrt = y - (error / 2.0) * x
        let zORt = yOrt |> ofVector |> crossProduct (xORt |> ofVector) |> toVector

        Matrix<float>.Build
            .DenseOfRowVectors(xORt.Normalize(2.0), yOrt.Normalize(2.0), zORt.Normalize(2.0))
        |> fromMatrix

module RotationMatrix3D =
    open Vector3D

    let zero = Matrix<float>.Build.DenseIdentity 3 |> Matrix3.Value

    let fromAxisAndAngle (Vector3D axis) angle =

        let len = axis.Norm(2.0)

        if axis <> Vector3D.zero.Get() && len < 1.0 - 1e-14 && len > 1.0 + 1e-14 then
            invalidArg "axis" "axis must be unit vector"

        let ux = axis.Item(0)
        let uy = axis.Item(1)
        let uz = axis.Item(2)

        let cosA = angle |> cos
        let cosInv = 1.0 - cosA
        let sinA = angle |> sin


        matrix
            [ [ cos angle + ux * ux * cosInv
                ux * uy * cosInv - uz * sinA
                ux * uz * cosInv + uy * sinA ]
              [ uy * ux * cosInv + uz * sinA
                cosA + uy * uy * cosInv
                uy * uz * cosInv - ux * sinA ]
              [ uz * ux * cosInv - uy * sinA
                uz * uy * cosA + ux * sinA
                cosA + uz * uz * cosInv ] ]
        |> Matrix3.Value

module Utils =
    let rec subSetsOf2<'t when 't: comparison> (set: 't Set) =
        if (set.Count < 2) then
            Set.empty
        elif (set.Count = 2) then
            set |> Set.singleton
        else
            let maximumElement = set |> Set.toSeq |> Seq.head
            let subset = set |> Set.remove maximumElement

            subset
            |> subSetsOf2
            |> Set.union (subset |> Set.map (fun el -> [ el; maximumElement ] |> Set.ofList))

    [<TailCall>]
    let rec private subSetsOf2Ag<'t when 't: comparison> (processed: 't Set) rest accumulator =
        if rest |> Set.count = 0 then
            accumulator
        else
            let maximumElement = rest |> Set.toSeq |> Seq.head

            subSetsOf2Ag
                (processed |> Set.add maximumElement)
                (rest |> Set.remove maximumElement)
                accumulator |> Set.union (processed |> Set.map (fun el -> [ el; maximumElement ] |> Set.ofList))

    let subSetsOf2Tail set = subSetsOf2Ag Set.empty set Set.empty
