namespace PhysicsSimulator

open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities

type RigidBodyKind =
    | Sphere of Sphere
    | Box of Box

type RigidBodyPrototype =
    { Kind: RigidBodyKind
      Collider: Collider

      Position: Vector3D
      Velocity: Vector3D
      Mass: Mass

      Yaw: float
      Pitch: float
      Roll: float
      AngularVelocity: Vector3D

      ElasticityCoeff: float
      StaticFrictionCoeff: float
      KineticFrictionCoeff: float
      UseGravity: bool }

module RigidBodyPrototype =
    let private defaultElasticityCoeff = 0.1
    let private defaultFrictionCoeff = 0.5

    let createDefault kind =
        let collider =
            match kind with
            | Sphere sphere -> sphere |> Collider.Sphere
            | Box box -> box |> Collider.Box

        { Kind = kind
          Collider = collider
          Position = Vector3D.zero
          Velocity = Vector3D.zero
          Mass = 1.0 |> Mass.Value
          Yaw = 0
          Pitch = 0
          Roll = 0
          AngularVelocity = Vector3D.zero
          ElasticityCoeff = defaultElasticityCoeff
          UseGravity = true
          StaticFrictionCoeff = defaultFrictionCoeff
          KineticFrictionCoeff = defaultFrictionCoeff + 0.1 }

    let createDefaultBox xSize ySize zSize =
        (xSize, ySize, zSize) |||> Box.create |> RigidBodyKind.Box |> createDefault

    let createDefaultCube size =
        size |> Box.createCube |> RigidBodyKind.Box |> createDefault

    let createDefaultSphere radius =
        { Sphere.Radius = radius } |> RigidBodyKind.Sphere |> createDefault

    let private validate prototype =
        match prototype.Mass, prototype.UseGravity with
        | Mass.Infinite, true ->
            invalidArg
                (nameof prototype.UseGravity)
                "objects with infinite mass cannot use gravity"
        | _ -> ()

    let internal build prototype id : SimulatorObject =
        validate prototype

        let inertiaMatrix =
            (match prototype.Kind with
             | Sphere sphere -> sphere.CreateRotationalInertia(prototype.Mass.GetValue)
             | Box box -> box.CreateRotationalInertia(prototype.Mass.GetValue))

        let rigidBody =
            RigidBody.create
                (RotationMatrix3D.fromYawPitchRoll prototype.Yaw prototype.Pitch prototype.Roll)
                prototype.Velocity
                prototype.ElasticityCoeff
                prototype.StaticFrictionCoeff
                prototype.KineticFrictionCoeff
                prototype.Mass
                prototype.Position
                inertiaMatrix

        { PhysicalObject = rigidBody |> RigidBody
          Collider = prototype.Collider
          Id = id }

    let getAABoundingBox (prototype: RigidBodyPrototype) : AABB =
        SimulatorObjectIdentifier.fromInt 0
        |> build prototype
        |> SimulatorObject.getAABoundingBox

module SpatialTreeBoundaries =
    open System

    let private aabbMinCorner (bb: AABB) =
        bb.CenterPosition - (bb.Size |> Box.toVector3D) / 2.0

    let private aabbMaxCorner (bb: AABB) =
        bb.CenterPosition + (bb.Size |> Box.toVector3D) / 2.0

    let private vecMin (a: Vector3D) (b: Vector3D) =
        Vector3D.create (min a.X b.X) (min a.Y b.Y) (min a.Z b.Z)

    let private vecMax (a: Vector3D) (b: Vector3D) =
        Vector3D.create (max a.X b.X) (max a.Y b.Y) (max a.Z b.Z)

    /// Union of prototype AABBs expanded by padding (static geometry) and motionMargin (dynamic travel).
    let fromPrototypeAABBs
        (prototypes: RigidBodyPrototype list)
        (padding: float)
        (motionMargin: Vector3D)
        : SpaceBoundaries
        =
        let expand = Vector3D.create padding padding padding + motionMargin

        let minCorner, maxCorner =
            prototypes
            |> List.map RigidBodyPrototype.getAABoundingBox
            |> List.fold
                (fun (minAcc, maxAcc) bb ->
                    aabbMinCorner bb |> vecMin minAcc,
                    aabbMaxCorner bb |> vecMax maxAcc)
                (Vector3D.create Double.MaxValue Double.MaxValue Double.MaxValue,
                 Vector3D.create Double.MinValue Double.MinValue Double.MinValue)

        { Min = minCorner - expand
          Max = maxCorner + expand }
