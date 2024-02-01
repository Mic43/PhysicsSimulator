namespace PhysicsSimulator.Utilities

open Vector3D

type Plane =
    { Normal: NormalVector
      DistanceFromOrigin: float }


module Plane =
    let create distance normal =

        { Normal = normal
          DistanceFromOrigin = distance }

    let inverted (plane: Plane) = {  Normal = -plane.Normal
                                     DistanceFromOrigin = -plane.DistanceFromOrigin }
