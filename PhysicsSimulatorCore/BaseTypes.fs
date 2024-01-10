namespace PhysicsSimulator

open MathNet.Numerics.LinearAlgebra
open Utils

type Matrix3 =
    private
    | Value of Matrix<float>

    member this.Get =
        match this with
        | Value v -> v

type Vector3D =
    private
    | Value of Vector<float>

    member this.Get =
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

  
    static member apply2 (f: Vector<float> -> Vector<float> -> Vector<float>) (v: Vector3D) (v2: Vector3D) =
        (v.Get, v2.Get) ||> f |> Value

    static member (+)(v1, v2) = (v1, v2) ||> Vector3D.apply2 (+)
    static member (-)(v1, v2) = (v1, v2) ||> Vector3D.apply2 (-)
    static member (*)(scalar: float, v: Vector3D) = scalar * v.Get |> Value
    static member (*)(v: Vector3D, scalar: float) = scalar * v
    static member (/)(v: Vector3D, scalar: float) = v * (1.0 / scalar)
    static member (~-)(v: Vector3D) = -v.Get |> Value


// static member op_Implicit(v: Vector<float>) = v |> Vector3D.Value

type Plane =
    { Normal: Vector3D
      DistanceFromOrigin: float }

type SetOf2<'t> =
    private
    | Value of List<'t> 

    member this.Get =
        match this with
        | Value v -> v

module SetOf2 =
    let create fst second = [ fst; second ] |> Value

    let ofList lst =
        match lst with
        | fst :: snd :: tail when tail.IsEmpty -> (fst, snd) ||> create
        | _ -> invalidArg "lst" "lst must be of size 2"

    let ofSet set = set |> Set.toList |> ofList

    let fst (set: SetOf2<'t>) = set.Get[0]
    let snd (set: SetOf2<'t>) = set.Get[1]

    let map f (set: SetOf2<'T>) = set.Get |> List.map f |> ofList
    let zip (set1: SetOf2<'T>) (set2: SetOf2<'V>) = set2.Get |> List.zip set1.Get |> ofList

    let zip3 (set1: SetOf2<'T>) (set2: SetOf2<'V>) (set3: SetOf2<'U>) =
        (set1.Get, set2.Get, set3.Get) |||> List.zip3 |> ofList


module Vector3D =
    let create x y z = vector [ x; y; z ] |> Vector3D.Value
    let zero = create 0.0 0.0 0.0
    let isZero v = v = zero
    let (|Vector3D|) (Vector3D.Value value) = value
    let ofVector (vec: Vector<float>) = vec |> Vector3D.Value
    let toVector (vec3d: Vector3D) = vec3d.Get

    let apply (f: Vector<float> -> Vector<float>) (v: Vector3D) = v |> toVector |> f |> ofVector

    let apply2 (f: Vector<float> -> Vector<float> -> Vector<float>) (v: Vector3D) (v2: Vector3D) =
        (f,v,v2) |||> Vector3D.apply2      
    let crossProduct (first: Vector3D) (second: Vector3D) =
        let x = (first.Y * second.Z) - (first.Z * second.Y)
        let y = (first.Z * second.X) - (first.X * second.Z)
        let z = (first.X * second.Y) - (first.Y * second.X)

        (x, y, z) |||> create

    let l2Norm (v: Vector3D) = v.Get.L2Norm()
    let normalized v = v |> apply (_.Normalize(2.0))

    let crossProductV (first) (second) =
        second |> ofVector |> crossProduct (first |> ofVector) |> toVector

    let dotProduct (v1: Vector3D) (v2: Vector3D) = v1.Get.DotProduct(v2.Get)

    let getOriented (rotationMatrix: Matrix3) (translationVector: Vector3D) (v: Vector3D) =
        v
        |> ((fun v -> v.Get * rotationMatrix.Get) >> (fun v -> v + translationVector.Get))
        |> ofVector

module Matrix3 =
    //open Vector3D
    let (|Matrix3|) (Value value) = value

    let ofMatrix (matrix: Matrix<float>) =
        if matrix.RowCount <> 3 || matrix.ColumnCount <> 3 then
            invalidArg "matrix" "matrix must be of size 3"

        matrix |> Matrix3.Value

    let orthonormalize (matrix: Matrix3) =

        let matrix = matrix.Get
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

        Matrix.Build.DenseOfColumns([ col0; col1; col2 ]) |> ofMatrix

module RotationMatrix3D =
    // open Vector3D
    let zero = Matrix<float>.Build.DenseIdentity 3 |> Matrix3.Value

    let fromAxisAndAngle (Vector3D.Vector3D axis) angle =

        let len = axis.Norm(2.0)

        if axis <> Vector3D.zero.Get && equals len 1.0 |> not then
            invalidArg "axis" "axis must be unit vector"

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

module Plane =
    let create distance normal =
        // let eps = 0.00001

        if normal |> Vector3D.toVector |> Vector.norm |> equals 1.0 |> not then
            invalidArg "normal" "normal vector must me normalized"

        { Normal = normal
          DistanceFromOrigin = distance }

    let invertNormal (plane: Plane) =
        { plane with
            Normal = plane.Normal |> Vector3D.apply (~-) }
