namespace PhysicsSimulator.Utilities

open MathNet.Numerics.LinearAlgebra

[<RequireQualifiedAccess>]
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

    static member apply2 (f: Vector<float> -> Vector<float> -> Vector<float>) (v: Vector3D) (v2: Vector3D) =
        (v.Get, v2.Get) ||> f |> Value

    static member (+)(v1, v2) = (v1, v2) ||> Vector3D.apply2 (+)    
    static member (-)(v1, v2) = (v1, v2) ||> Vector3D.apply2 (-)
    static member (*)(v1: Vector3D, v2: Vector3D) = v1.Get * v2.Get
    static member (*)(scalar: float, v: Vector3D) = scalar * v.Get |> Value
    static member (*)(v: Vector3D, scalar: float) = scalar * v
    static member (/)(v: Vector3D, scalar: float) = v * (1.0 / scalar)
    static member (~-)(v: Vector3D) = -v.Get |> Value

[<RequireQualifiedAccess>]
type NormalVector =
    private
    | Value of Vector3D

    member this.Get =
        match this with
        | Value v -> v

    static member (~-)(NormalVector.Value v) = -v |> Value

module Vector3D =
    let create x y z = vector [ x; y; z ] |> Vector3D.Value
    let zero = create 0.0 0.0 0.0
    let (|Vector3D|) (Vector3D.Value value) = value
    let ofVector (vec: Vector<float>) = vec |> Vector3D.Value
    let toVector (vec3d: Vector3D) = vec3d.Get

    let apply (f: Vector<float> -> Vector<float>) (v: Vector3D) = v |> toVector |> f |> ofVector

    let apply2 (f: Vector<float> -> Vector<float> -> Vector<float>) (v: Vector3D) (v2: Vector3D) =
        (f, v, v2) |||> Vector3D.apply2

    let crossProduct (first: Vector3D) (second: Vector3D) =
        let x = (first.Y * second.Z) - (first.Z * second.Y)
        let y = (first.Z * second.X) - (first.X * second.Z)
        let z = (first.X * second.Y) - (first.Y * second.X)

        (x, y, z) |||> create

    let l2Norm (v: Vector3D) = v.Get.L2Norm()

    let normalized v =
        v |> apply (_.Normalize(2.0)) |> NormalVector.Value

    let crossProductV (first) (second) =
        second |> ofVector |> crossProduct (first |> ofVector) |> toVector

    let dotProduct (v1: Vector3D) (v2: Vector3D) = v1.Get.DotProduct(v2.Get)
    let isZero epsilon (v: Vector3D) = v |> l2Norm |> equals epsilon 0.0
    let areParallel epsilon v1 v2 = v1 |> crossProduct v2 |> isZero epsilon

module NormalVector =
    let internal createUnsafe v = v |> NormalVector.Value

    let create epsilon v =
        if v |> Vector3D.l2Norm |> equals epsilon 1.0 |> not then
            invalidArg "v" "normal vector must me normalized"

        v |> NormalVector.Value

    let toVector3D (normal: NormalVector) = normal.Get
