namespace PhysicsSimulator.Entities

open PhysicsSimulator.Utilities

type Collider =
    | Sphere of Sphere
    | Box of Box

module Collider =
    let createSphere = Sphere.create >> Sphere

    let createBox x y z = (Box.create x y z) |> Collider.Box
