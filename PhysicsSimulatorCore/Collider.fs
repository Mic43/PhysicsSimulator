namespace PhysicsSimulator

type Sphere = { Radius: float }

type Box =
    { XSize: float
      YSize: float
      ZSize: float }

type Collider =
    | Sphere of Sphere
    | Box of Box

module Collider =
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
        |> Box

    let getVertices (box: Box) =                
        let t =
            [ box.XSize; box.YSize; box.ZSize ]
            |> List.map (fun v -> v / 2.0)
            |> List.map (fun e -> [ id; (~-) ] |> List.map (fun op -> op e))

        seq {
            for x in t[0] do
                for y in t[1] do
                    for z in t[2] do
                        yield Vector3D.create x  y  z
        }
