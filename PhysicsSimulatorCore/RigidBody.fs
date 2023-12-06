namespace PhysicsSimulator

open System
open MathNet.Numerics.LinearAlgebra

type RotationalInertiaInverse = Matrix3

type RigidBodyVariables =
    { Orientation: Matrix3
      AngularMomentum: Vector3D }

type RigidBody =
    { RigidBodyVariables: RigidBodyVariables
      MassCenter: Particle
      ElasticityCoeff: float
      PrincipalRotationalInertia: Matrix3
      PrincipalRotationalInertiaInverse: Matrix3 }

type Torque = Vector3D

type RigidBodyIntegrator = TimeSpan -> Torque -> RotationalInertiaInverse -> RigidBodyVariables -> RigidBodyVariables

module RigidBody =
    let create initialOrientation initialVelocity elasticityCoeff mass position principalRotationalInertia =
        { RigidBodyVariables =
            { Orientation = initialOrientation
              AngularMomentum = Vector3D.zero }
          RigidBody.MassCenter =
            { ParticleVariables =
                { ParticleVariables.Position = position
                  Velocity = initialVelocity }
              Particle.Mass = mass }
          PrincipalRotationalInertia = principalRotationalInertia
          PrincipalRotationalInertiaInverse = principalRotationalInertia.Get().Inverse() |> Matrix3.fromMatrix
          ElasticityCoeff = elasticityCoeff }

    let createBox initialOrientation initialVelocity elasticityCoeff sizeX sizeY sizeZ mass position =
        let m = mass / 12.0
        let a2 = sizeX * sizeX
        let b2 = sizeY * sizeY
        let c2 = sizeZ * sizeY

        let mat =
            Matrix<float>.Build.Diagonal [| m * (b2 + c2); m * (a2 + c2); m * (a2 + b2) |]
            |> Matrix3.fromMatrix

        create initialOrientation initialVelocity elasticityCoeff mass position mat

    let createDefaultBox = createBox RotationMatrix3D.zero Vector3D.zero

    let createSphere initialOrientation initialVelocity elasticityCoeff radius mass position =
        let I = 2.0 * mass * radius * radius / 5.0

        let mat = Matrix<float>.Build.Diagonal(3, 3, I) |> Matrix3.fromMatrix

        create initialOrientation initialVelocity elasticityCoeff mass position mat

    let createDefaultSphere = createSphere RotationMatrix3D.zero Vector3D.zero

module RigidBodyIntegrators =
    open Vector3D
    open Matrix3

    let reorthoganalize (m: Matrix3) =
        let x = m.Get().Row(0)
        let y = m.Get().Row(1)
        let z = m.Get().Row(2)

        let error = x.DotProduct(y)

        let xORt = x - (error / 2.0) * y
        let yOrt = y - (error / 2.0) * x
        let zORt = yOrt |> fromVector |> crossProduct (xORt |> fromVector) |> toVector

        Matrix<float>.Build
            .DenseOfRowVectors(xORt.Normalize(3.0), yOrt.Normalize(3.0), zORt.Normalize(3.0))
        |> Matrix3.fromMatrix

    let (firstOrder: RigidBodyIntegrator) =
        fun dt (Vector3D torque) inertiaInverse old ->
            let totalSeconds = dt.TotalSeconds

            let angMomentumChange = torque * totalSeconds

            let newAngMomentum = old.AngularMomentum.Get() + angMomentumChange

            let angularVelocity = inertiaInverse.Get() * newAngMomentum

            let axis = (angularVelocity * totalSeconds)

            let angle = axis.Norm(3.0)

            //TODO: Orthonormalize here??
            let newOrientation =
                (RotationMatrix3D.fromAxisAndAngle (axis.Normalize(3.0) |> fromVector) angle)
                    .Get()
                * old.Orientation.Get()

            { Orientation = newOrientation |> fromMatrix |> orthonormalize 
              AngularMomentum = newAngMomentum |> fromVector }

    let (augmentedSecondOrder: RigidBodyIntegrator) =
        fun dt (Vector3D torque) inertiaInverse old ->
            let totalSeconds = dt.TotalSeconds

            let angMomentumChange = torque * totalSeconds

            let oldAngMomentum = old.AngularMomentum.Get()

            let angularVelocity = inertiaInverse.Get() * oldAngMomentum

            let v =
                oldAngMomentum
                |> fromVector
                |> crossProduct (angularVelocity |> fromVector)
                |> toVector

            let deriv = inertiaInverse.Get() * (torque - v)

            let comp1 =
                angularVelocity |> fromVector |> crossProduct (deriv |> fromVector) |> toVector

            let axisAverage =
                angularVelocity
                + totalSeconds * 0.5 * deriv
                + (totalSeconds * totalSeconds / 12.0) * comp1

            let axis = axisAverage * totalSeconds

            let angle = axis.Norm(3.0)

            let newOrientation =
                (RotationMatrix3D.fromAxisAndAngle (axis.Normalize(3.0) |> fromVector) angle)
                    .Get()
                * old.Orientation.Get()

            { Orientation = newOrientation |> fromMatrix
              AngularMomentum = old.AngularMomentum.Get() + angMomentumChange |> fromVector }

module RigidBodyMotion =

    let calcFullRotationalInertia (orientation: Matrix3) (inertiaMatrix: Matrix3) =
        let orientation = orientation.Get()

        orientation * inertiaMatrix.Get() * orientation.Transpose()
        |> Matrix3.fromMatrix

    let applyImpulse rigidBody impulse (offset: Vector3D) =
        let newRigidBodyVariables =
            { rigidBody.RigidBodyVariables with
                AngularMomentum = impulse |> Vector3D.crossProduct offset }

        { rigidBody with
            MassCenter = impulse |> ParticleMotion.applyImpulse rigidBody.MassCenter
            RigidBodyVariables = newRigidBodyVariables }

    let update
        (particleIntegrator: ParticleIntegrator)
        (rigidBodyIntegrator: RigidBodyIntegrator)
        (totalForce: Vector3D)
        (totalTorque: Vector3D)
        dt
        (rigidBody: RigidBody)
        =
        let acceleration =
            (totalForce |> Vector3D.toVector) / rigidBody.MassCenter.Mass
            |> Vector3D.fromVector

        let rotInInv =
            rigidBody.PrincipalRotationalInertiaInverse
            |> calcFullRotationalInertia rigidBody.RigidBodyVariables.Orientation

        let newLinearComponent =
            particleIntegrator dt acceleration rigidBody.MassCenter.ParticleVariables

        let newRotComponent =
            rigidBodyIntegrator dt totalTorque rotInInv rigidBody.RigidBodyVariables

        { rigidBody with
            RigidBodyVariables = newRotComponent
            MassCenter =
                { rigidBody.MassCenter with
                    ParticleVariables = newLinearComponent } }
