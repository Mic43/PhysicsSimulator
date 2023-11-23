namespace PhysicsSimulator

open System
open MathNet.Numerics.LinearAlgebra


type RotationalInertiaInverse = Matrix3

type RigidBodyVariables =
    { Orientation: Matrix3
      AngularMomentum: Vector3D }

type RigidBody =
    private
        { Orientation: Matrix3
          MassCenter: Particle
          RotationalInertia: Matrix3
          AngularMomentum: Vector3D }

type Torque = Vector3D
type RigidBodyIntegrator = TimeSpan -> Torque -> RotationalInertiaInverse -> RigidBodyVariables -> RigidBodyVariables

module RigidBody =
    let createBox initialOrientation initialVelocity sizeX sizeY sizeZ mass position =
        let m = mass / 12.0
        let a2 = sizeX * sizeX
        let b2 = sizeY * sizeY
        let c2 = sizeZ * sizeY

        { Orientation = initialOrientation
          RigidBody.MassCenter =
            { ParticleVariables =
                { ParticleVariables.Position = position
                  Velocity = initialVelocity }
              Particle.Mass = mass }
          RotationalInertia =
            Matrix<float>.Build.Diagonal
                [| m * (b2 + c2)
                   m * (a2 + c2)
                   m * (a2 + b2) |]
            |> Matrix3.fromMatrix
          AngularMomentum = Vector3D.zero }

    let createDefaultBox =
        createBox RotationMatrix3D.zero Vector3D.zero

    let createSphere initialOrientation initialVelocity radius mass position =
        let I = 2.0 * mass * radius * radius / 5.0

        { Orientation = initialOrientation
          RigidBody.MassCenter =
            { ParticleVariables =
                { ParticleVariables.Position = position
                  Velocity = initialVelocity }
              Particle.Mass = mass }
          RotationalInertia =
            Matrix<float>.Build.Diagonal (3, 3, I)
            |> Matrix3.fromMatrix
          AngularMomentum = Vector3D.zero }

    let createDefaultSphere =
        createSphere RotationMatrix3D.zero Vector3D.zero

module RigidBodyIntegrators =
    open Vector3D
    open Matrix3

    let (firstOrder: RigidBodyIntegrator) =
        fun dt (Vector3D torque) inertiaInverse old ->
            let milliseconds = dt.TotalMilliseconds

            let angMomentumChange =
                torque * milliseconds

            let newAngMomentum =
                old.AngularMomentum.Get() + angMomentumChange

            let angularVelocity =
                inertiaInverse.Get() * newAngMomentum

            let angle = angularVelocity * milliseconds

            let newOrientation =
                (RotationMatrix3D.fromAxisAndAngle (angle |> fromVector) (angle.L2Norm()))
                    .Get()
                * old.Orientation.Get()

            { RigidBodyVariables.Orientation = newOrientation |> fromMatrix
              AngularMomentum = newAngMomentum |> fromVector }

    let (augmentedSecondOrder: RigidBodyIntegrator) =
        fun dt (Vector3D torque) inertiaInverse old ->
            let milliseconds = dt.TotalMilliseconds

            let angMomentumChange =
                torque * milliseconds

            let oldAngMomentum =
                old.AngularMomentum.Get()

            let angularVelocity =
                inertiaInverse.Get() * oldAngMomentum

            let deriv =
                inertiaInverse.Get()
                * (torque
                   - angularVelocity.DotProduct(oldAngMomentum))

            let angAverage =
                angularVelocity
                + milliseconds * 0.5 * deriv
                + milliseconds * milliseconds / 12.0
                  * deriv.DotProduct(angularVelocity)

            let angle = angAverage * milliseconds

            let newOrientation =
                (RotationMatrix3D.fromAxisAndAngle (angle |> fromVector) (angle.L2Norm()))
                    .Get()
                * old.Orientation.Get()

            { RigidBodyVariables.Orientation = newOrientation |> fromMatrix
              AngularMomentum =
                old.AngularMomentum.Get() + angMomentumChange
                |> fromVector }
