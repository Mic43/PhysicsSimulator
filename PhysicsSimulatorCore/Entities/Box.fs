namespace PhysicsSimulator.Entities

open MathNet.Numerics.LinearAlgebra
open PhysicsSimulator.Utilities
open Vector3D

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
    let private vertices =
        [ (-1.0, -1.0, -1.0)
          (-1.0, 1.0, -1.0)
          (1.0, 1.0, -1.0)
          (1.0, -1.0, -1.0)
          (-1.0, -1.0, 1.0)
          (-1.0, 1.0, 1.0)
          (1.0, 1.0, 1.0)
          (1.0, -1.0, 1.0) ]
        |> List.map ((|||>) >> (fun f -> create |> f))

    let private faces =
        [ [ 0; 1; 2; 3 ]
          [ 7; 6; 5; 4 ]
          [ 5; 6; 2; 1 ]
          [ 0; 3; 7; 4 ]
          [ 6; 7; 3; 2 ]
          [ 4; 5; 1; 0 ] ]

    let private normals =
        [ (0.0, 0.0, -1.0)
          (0.0, 0.0, 1.0)
          (0.0, 1.0, 0.0)
          (0.0, -1.0, 0.0)
          (1.0, 0.0, 0.0)
          (-1.0, 0.0, 0.0) ]
        |> List.map (fun v -> v |||> Vector3D.create)

    let create xSize ySize zSize =
        do throwIfNegative xSize (nameof xSize)
        do throwIfNegative ySize (nameof ySize)
        do throwIfNegative zSize (nameof zSize)

        { XSize = xSize
          YSize = ySize
          ZSize = zSize }

    let ofVector3D (vector: Vector3D) =
        (vector.X, vector.Y, vector.Z) |||> create

    let createCube size = (size, size, size) |||> create

    let toVector3D box =
        (box.XSize, box.YSize, box.ZSize) |||> Vector3D.create

    let getVertices (box: Box) =
        let t =
            [ box.XSize; box.YSize; box.ZSize ]
            |> List.map (fun v -> v / 2.0)
            |> List.map (fun e -> [ id; (~-) ] |> List.map (fun op -> op e))

        seq {
            for x in t[0] do
                for y in t[1] do
                    for z in t[2] do
                        yield Vector3D.create x y z
        }

    let getFaces (box: Box) : Face list =
        let scaleVector =
            (box.XSize / 2.0, box.YSize / 2.0, box.ZSize / 2.0)
            |||> Vector3D.create
            |> toVector

        normals
        |> List.mapi (fun i normal ->
            { Vertices =
                faces[i]
                |> List.map ((fun i -> vertices[i].Get) >> (_.PointwiseMultiply(scaleVector)) >> ofVector)
              Normal = normal |> NormalVector.createUnsafe })

    let findFaceByNormal normal (faces: Face seq) =
        faces |> Seq.find (fun face -> face.Normal = normal)

    let findAdjacentFaces epsilon (boxFaces: Face seq) (targetFace: Face) =
        boxFaces // take only faces that have their normals anti parallel to given face normal
        |> Seq.filter (
            (_.Normal.Get)
            >> (dotProduct targetFace.Normal.Get)
            >> abs
            >> (equals epsilon 1.0)
            >> not
        )
