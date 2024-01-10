namespace PhysicsSimulator.Entities

open PhysicsSimulator.Utilities

open Constants
type Sphere = { Radius: float }

type Box =
    { XSize: float
      YSize: float
      ZSize: float }

type Collider =
    | Sphere of Sphere
    | Box of Box

type Face =
    { Vertices: Vector3D seq
      Normal: Vector3D }

module Face =
    let toPlane face =
        face.Normal
        |> Plane.create ((face.Vertices |> Seq.head).Get.DotProduct(face.Normal.Get))

module Collider =
    let private vertices =
        [ (-1.0, -1.0, -1.0) |||> Vector3D.create
          (-1.0, 1.0, -1.0) |||> Vector3D.create
          (1.0, 1.0, -1.0) |||> Vector3D.create
          (1.0, -1.0, -1.0) |||> Vector3D.create
          (-1.0, -1.0, 1.0) |||> Vector3D.create
          (-1.0, 1.0, 1.0) |||> Vector3D.create
          (1.0, 1.0, 1.0) |||> Vector3D.create
          (1.0, -1.0, 1.0) |||> Vector3D.create ]

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

    let private throwIfNegative value name =
        if value < 0.0 then
            "Must be positive" |> invalidArg name

    let createSphere radius =
        do throwIfNegative radius (nameof (radius))

        { Radius = radius } |> Sphere

    let createBox xSize ySize zSize =
        do throwIfNegative xSize (nameof xSize)
        do throwIfNegative ySize (nameof ySize)
        do throwIfNegative zSize (nameof zSize)

        { XSize = xSize
          YSize = ySize
          ZSize = zSize }

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

    let getFaces (box: Box) : Face seq =
        let scaleVector =
            (box.XSize / 2.0, box.YSize / 2.0, box.ZSize / 2.0)
            |||> Vector3D.create
            |> Vector3D.toVector

        normals
        |> List.mapi (fun i normal ->
            { Vertices =
                faces[i]
                |> List.map (
                    (fun i -> vertices[i].Get)
                    >> (_.PointwiseMultiply(scaleVector))
                    >> Vector3D.ofVector
                )
              Normal = normal })
        |> List.toSeq

    let findFaceByNormal normal faces =
        faces |> Seq.find (fun face -> face.Normal = normal)

    let findAdjacentFaces boxFaces targetFace =
        boxFaces
        |> Seq.filter (fun f -> f.Normal - targetFace.Normal |> Vector3D.l2Norm |> (equals 0.0))
        |> Seq.filter (fun face ->
            face.Vertices
            |> Seq.allPairs targetFace.Vertices
            |> Seq.exists (fun (v1, v2) -> v1 - v2 |> Vector3D.l2Norm |> (equals 0.0)))
//face.Normal |> Vector3D.crossProduct targetFace.Normal |> abs > epsilon)
