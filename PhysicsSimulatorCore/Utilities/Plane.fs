namespace PhysicsSimulator.Utilities

open Vector3D

type Plane =
    { Normal: NormalVector
      DistanceFromOrigin: float }


module Plane =
    let create distance normal =

        { Normal = normal
          DistanceFromOrigin = distance }

    let invertNormal (plane: Plane) = { plane with Normal = -plane.Normal }
