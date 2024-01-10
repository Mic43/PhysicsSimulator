namespace PhysicsSimulator.Utilities

open Vector3D
type Plane =
    { Normal: Vector3D
      DistanceFromOrigin: float }
    
    
module Plane =
    let create distance normal =
        // let eps = 0.00001

        if normal |> l2Norm |> equals 1.0 |> not then
            invalidArg "normal" "normal vector must me normalized"

        { Normal = normal
          DistanceFromOrigin = distance }

    let invertNormal (plane: Plane) =
        { plane with
            Normal = plane.Normal |> apply (~-) }

