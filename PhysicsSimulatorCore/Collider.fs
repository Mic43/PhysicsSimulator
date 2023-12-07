namespace PhysicsSimulator

open FSharpPlus

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
        do throwIfNegative radius (nameof(radius))

        { Radius = radius } |> Sphere

    let createBox xSize ySize zSize =
        do throwIfNegative xSize (nameof(xSize))
        do throwIfNegative ySize (nameof(ySize))
        do throwIfNegative zSize (nameof(zSize))

        { XSize = xSize
          YSize = ySize
          ZSize = zSize } |> Box
