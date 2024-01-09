namespace PhysicsSimulator

open System
open MathNet.Numerics.LinearAlgebra

type RotationalInertiaInverse = Matrix3

type RigidBodyVariables =
    { Orientation: Matrix3
      AngularMomentum: Vector3D }

type RigidBody =
    { Variables: RigidBodyVariables
      MassCenter: Particle
      ElasticityCoeff: float
      FrictionCoeff: float
      PrincipalRotationalInertia: Matrix3
      PrincipalRotationalInertiaInverse: Matrix3 }

    static member CalcFullRotationalInertia (orientation: Matrix3) (inertiaMatrix: Matrix3) =
        let orientation = orientation.Get
        orientation * inertiaMatrix.Get * orientation.Transpose() |> Matrix3.fromMatrix

    member this.CalcRotationalInertia() =
        RigidBody.CalcFullRotationalInertia this.Variables.Orientation this.PrincipalRotationalInertia

    member this.CalcRotationalInertiaInverse() =
        RigidBody.CalcFullRotationalInertia this.Variables.Orientation this.PrincipalRotationalInertiaInverse

    member this.GetMassCenterPosition = this.MassCenter.Variables.Position

    member this.CalcAngularVelocity() =
        this.CalcRotationalInertiaInverse().Get * this.Variables.AngularMomentum.Get
        |> Vector3D.ofVector

type Torque = Vector3D

type RigidBodyIntegrator = TimeSpan -> Torque -> RotationalInertiaInverse -> RigidBodyVariables -> RigidBodyVariables

module RigidBody =
    let create
        initialOrientation
        initialVelocity
        elasticityCoeff
        frictionCoeff
        mass
        position
        principalRotationalInertia
        =
        if elasticityCoeff < 0.0 || elasticityCoeff > 1.0 then
            "Invalid value " |> invalidArg (nameof elasticityCoeff)

        if frictionCoeff < 0.0 || frictionCoeff > 1.0 then
            "Invalid value " |> invalidArg (nameof frictionCoeff)

        { Variables =
            { Orientation = initialOrientation
              AngularMomentum = Vector3D.zero }
          RigidBody.MassCenter =
            { Variables =
                { ParticleVariables.Position = position
                  Velocity = initialVelocity }
              Particle.Mass = mass }
          PrincipalRotationalInertia = principalRotationalInertia
          PrincipalRotationalInertiaInverse = principalRotationalInertia.Get.Inverse() |> Matrix3.fromMatrix
          ElasticityCoeff = elasticityCoeff
          FrictionCoeff = frictionCoeff }

    let createBox initialOrientation initialVelocity elasticityCoeff frictionCoeff sizeX sizeY sizeZ mass position =
        let m = mass / 12.0
        let a2 = sizeX * sizeX
        let b2 = sizeY * sizeY
        let c2 = sizeZ * sizeY

        let mat =
            Matrix<float>.Build.Diagonal [| m * (b2 + c2); m * (a2 + c2); m * (a2 + b2) |]
            |> Matrix3.fromMatrix

        create initialOrientation initialVelocity elasticityCoeff frictionCoeff mass position mat

    let createDefaultBox = createBox RotationMatrix3D.zero Vector3D.zero

    let createSphere initialOrientation initialVelocity elasticityCoeff frictionCoeff radius mass position =
        let I = 2.0 * mass * radius * radius / 5.0

        let mat = Matrix<float>.Build.Diagonal(3, 3, I) |> Matrix3.fromMatrix

        create initialOrientation initialVelocity elasticityCoeff frictionCoeff mass position mat

    let createDefaultSphere = createSphere RotationMatrix3D.zero Vector3D.zero

module RigidBodyIntegrators =
    open Vector3D
    open Matrix3

    let (firstOrder: RigidBodyIntegrator) =
        fun dt torque inertiaInverse old ->
            let totalSeconds = dt.TotalSeconds

            let angMomentumChange = torque * totalSeconds

            let newAngMomentum = old.AngularMomentum + angMomentumChange

            let angularVelocity = inertiaInverse.Get * newAngMomentum.Get |> ofVector

            let axis = angularVelocity * totalSeconds

            let angle = axis |> l2Norm

            let newOrientation =
                (angle |> RotationMatrix3D.fromAxisAndAngle (axis |> normalized)).Get
                * old.Orientation.Get

            { Orientation = newOrientation |> fromMatrix |> orthonormalize
              AngularMomentum = newAngMomentum }

    let (augmentedSecondOrder: RigidBodyIntegrator) =
        fun dt (Vector3D torque) inertiaInverse old ->
            let totalSeconds = dt.TotalSeconds

            let angMomentumChange = torque * totalSeconds

            let oldAngMomentum = old.AngularMomentum.Get

            let angularVelocity = inertiaInverse.Get * oldAngMomentum

            let v = oldAngMomentum |> crossProductV angularVelocity

            let deriv = inertiaInverse.Get * (torque - v)

            let comp1 = angularVelocity |> crossProductV deriv

            let axisAverage =
                angularVelocity
                + totalSeconds * 0.5 * deriv
                + (totalSeconds * totalSeconds / 12.0) * comp1

            let axis = axisAverage * totalSeconds

            let angle = axis.Norm(2.0)

            let newOrientation =
                (RotationMatrix3D.fromAxisAndAngle (axis.Normalize(2.0) |> ofVector) angle).Get
                * old.Orientation.Get

            { Orientation = newOrientation |> fromMatrix |> orthonormalize
              AngularMomentum = old.AngularMomentum + (angMomentumChange |> ofVector) }

module RigidBodyMotion =

    let applyImpulse rigidBody impulse (offset: Vector3D) =

        { rigidBody with
            MassCenter = impulse |> ParticleMotion.applyImpulse rigidBody.MassCenter
            Variables.AngularMomentum = rigidBody.Variables.AngularMomentum + (impulse |> Vector3D.crossProduct offset) }

    let update
        (particleIntegrator: ParticleIntegrator)
        (rigidBodyIntegrator: RigidBodyIntegrator)
        (totalForce: Vector3D)
        (totalTorque: Vector3D)
        dt
        (rigidBody: RigidBody)
        =
        let acceleration = totalForce / rigidBody.MassCenter.Mass

        let newLinearComponent =
            particleIntegrator dt acceleration rigidBody.MassCenter.Variables

        let newRotComponent =
            rigidBodyIntegrator dt totalTorque (rigidBody.CalcRotationalInertiaInverse()) rigidBody.Variables

        { rigidBody with
            Variables = newRotComponent
            MassCenter.Variables = newLinearComponent }
