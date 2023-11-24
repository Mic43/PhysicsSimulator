namespace PhysicsSimulator

open System
open MathNet.Numerics.LinearAlgebra

type RotationalInertiaInverse = Matrix3

type RigidBodyVariables =
    { Orientation: Matrix3
      AngularMomentum: Vector3D }

type RigidBody =
    private
        { RigidBodyVariables: RigidBodyVariables
          MassCenter: Particle
          PrincipalRotationalInertia: Matrix3
          PrincipalRotationalInertiaInverse: Matrix3 }

type Torque = Vector3D
type RigidBodyIntegrator = TimeSpan -> Torque -> RotationalInertiaInverse -> RigidBodyVariables -> RigidBodyVariables

module RigidBody =
    let create initialOrientation initialVelocity mass position principalRotationalInertia =
        { RigidBodyVariables =
            { Orientation = initialOrientation
              AngularMomentum = Vector3D.zero }
          RigidBody.MassCenter =
            { ParticleVariables =
                { ParticleVariables.Position = position
                  Velocity = initialVelocity }
              Particle.Mass = mass }
          PrincipalRotationalInertia = principalRotationalInertia
          PrincipalRotationalInertiaInverse =
            principalRotationalInertia.Get().Inverse()
            |> Matrix3.fromMatrix }

    let createBox initialOrientation initialVelocity sizeX sizeY sizeZ mass position =
        let m = mass / 12.0
        let a2 = sizeX * sizeX
        let b2 = sizeY * sizeY
        let c2 = sizeZ * sizeY

        let mat =
            Matrix<float>.Build.Diagonal
                [| m * (b2 + c2)
                   m * (a2 + c2)
                   m * (a2 + b2) |]
            |> Matrix3.fromMatrix

        create initialOrientation initialVelocity mass position mat

    let createDefaultBox =
        createBox RotationMatrix3D.zero Vector3D.zero

    let createSphere initialOrientation initialVelocity radius mass position =
        let I = 2.0 * mass * radius * radius / 5.0

        let mat =
            Matrix<float>.Build.Diagonal (3, 3, I)
            |> Matrix3.fromMatrix

        create initialOrientation initialVelocity mass position mat

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
                * (torque - oldAngMomentum
                   |> fromVector
                   |> crossProduct (angularVelocity |> fromVector)
                   |> toVector)

            let angAverage =
                angularVelocity
                + milliseconds * 0.5 * deriv
                + milliseconds * milliseconds / 12.0
                  * angularVelocity
                |> fromVector
                |> crossProduct (deriv |> fromVector)

            let angle =
                (angAverage |> toVector) * milliseconds

            let newOrientation =
                (RotationMatrix3D.fromAxisAndAngle (angle |> fromVector) (angle.L2Norm()))
                    .Get()
                * old.Orientation.Get()

            { RigidBodyVariables.Orientation = newOrientation |> fromMatrix
              AngularMomentum =
                old.AngularMomentum.Get() + angMomentumChange
                |> fromVector }

module Motion =
    let update
        (particleIntegrator: ParticleIntegrator)
        (rigidBodyIntegrator: RigidBodyIntegrator)
        (totalForce: Vector3D)
        (totalTorque: Vector3D)
        dt
        (rigidBody: RigidBody)
        =
        let acceleration =
            (totalForce |> Vector3D.toVector)
            / rigidBody.MassCenter.Mass
            |> Vector3D.fromVector

        let orientation =
            rigidBody.RigidBodyVariables.Orientation.Get()

        let rotInInv =
            orientation
            * rigidBody.PrincipalRotationalInertiaInverse.Get()
            * orientation.Transpose()
            |> Matrix3.fromMatrix

        let newLinearComponent =
            particleIntegrator dt acceleration rigidBody.MassCenter.ParticleVariables

        let newRotComponent =
            rigidBodyIntegrator dt totalTorque rotInInv rigidBody.RigidBodyVariables


        { rigidBody with
            RigidBodyVariables = newRotComponent
            MassCenter = { rigidBody.MassCenter with ParticleVariables = newLinearComponent } }
