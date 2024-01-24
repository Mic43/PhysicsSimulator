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
      FrictionCoeff: float
      UseGravity: bool }

module RigidBodyPrototype =    
    let private defaultElasticityCoeff = 0.9
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
          FrictionCoeff = defaultFrictionCoeff
          UseGravity = false }

    let createDefaultCube size =
        size |> Box.createCube |> RigidBodyKind.Box |> createDefault

    let createDefaultSphere radius =
        { Sphere.Radius = radius } |> RigidBodyKind.Sphere |> createDefault

    let internal build prototype id : SimulatorObject =
        let inertiaMatrix =
            (match prototype.Kind with
             | Sphere sphere -> sphere.CreateRotationalInertia(prototype.Mass.GetValue())
             | Box box -> box.CreateRotationalInertia(prototype.Mass.GetValue()))

        let rigidBody =
            RigidBody.create
                (RotationMatrix3D.fromYawPitchRoll prototype.Yaw prototype.Pitch prototype.Roll)
                prototype.Velocity
                prototype.ElasticityCoeff
                prototype.FrictionCoeff
                (prototype.Mass.GetValue())
                prototype.Position
                inertiaMatrix

        { PhysicalObject = rigidBody |> RigidBody
          Collider = prototype.Collider
          Id = id }
