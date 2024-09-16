namespace PhysicsSimulator.Entities

open System
open PhysicsSimulator.Utilities

type RigidBodyVariables =
    { Orientation: Matrix3
      AngularMomentum: Vector3D }

type RigidBody =
    { Variables: RigidBodyVariables
      MassCenter: Particle
      ElasticityCoeff: float
      StaticFrictionCoeff: float
      KineticFrictionCoeff: float
      PrincipalRotationalInertia: Matrix3
      PrincipalRotationalInertiaInverse: Matrix3 }

    static member CalcFullRotationalInertia (orientation: Matrix3) (inertiaMatrix: Matrix3) =
        orientation * inertiaMatrix * (orientation |> Matrix3.transposed)

    member this.CalcRotationalInertia() =
        RigidBody.CalcFullRotationalInertia this.Variables.Orientation this.PrincipalRotationalInertia

    member this.CalcRotationalInertiaInverse() =
        RigidBody.CalcFullRotationalInertia this.Variables.Orientation this.PrincipalRotationalInertiaInverse

    member this.MassCenterPosition = this.MassCenter.Variables.Position
    member this.MassCenterVelocity = this.MassCenter.Variables.Velocity

    member this.CalcAngularVelocity() =
        this.CalcRotationalInertiaInverse() * this.Variables.AngularMomentum

    member this.Axes =
        [ (1.0, 0.0, 0.0); (0.0, 1.0, 0.0); (0.0, 0.0, 1.0) ]
        |> List.map (
            (|||>)
            >> (fun f -> Vector3D.create |> f)
            >> (GraphicsUtils.toWorldCoordinates this.Variables.Orientation Vector3D.zero)
            >> Vector3D.normalized
        )

type RigidBodyIntegrator = TimeSpan -> Torque -> RotationalInertiaInverse -> RigidBodyVariables -> RigidBodyVariables

module internal RigidBody =
    let getAxes (rigidBody: RigidBody) = rigidBody.Axes

    let create
        initialOrientation
        initialVelocity
        elasticityCoeff
        staticFrictionCoeff
        kineticFrictionCoeff
        mass
        position
        principalRotationalInertia
        =
        if elasticityCoeff < 0.0 || elasticityCoeff > 1.0 then
            "Invalid value " |> invalidArg (nameof elasticityCoeff)

        if staticFrictionCoeff < 0.0 then
            "Invalid value " |> invalidArg (nameof staticFrictionCoeff)

        if kineticFrictionCoeff < 0.0 then
            "Invalid value " |> invalidArg (nameof staticFrictionCoeff)

        { Variables =
            { Orientation = initialOrientation
              AngularMomentum = Vector3D.zero }
          RigidBody.MassCenter =
            { Variables =
                { ParticleVariables.Position = position
                  Velocity = initialVelocity }
              Particle.Mass = mass }
          PrincipalRotationalInertia = principalRotationalInertia
          PrincipalRotationalInertiaInverse = principalRotationalInertia |> Matrix3.inverted
          ElasticityCoeff = elasticityCoeff
          StaticFrictionCoeff = staticFrictionCoeff
          KineticFrictionCoeff = kineticFrictionCoeff }

    let createBox
        initialOrientation
        initialVelocity
        elasticityCoeff
        staticFrictionCoeff
        kineticFrictionCoeff
        sizeX
        sizeY
        sizeZ
        mass
        position
        =
        let box =
            { XSize = sizeX
              YSize = sizeY
              ZSize = sizeZ }

        create
            initialOrientation
            initialVelocity
            elasticityCoeff
            staticFrictionCoeff
            kineticFrictionCoeff
            mass
            position
            (box.CreateRotationalInertia(mass.GetValue))

    let createDefaultBox = createBox RotationMatrix3D.zero Vector3D.zero

    let createSphere
        initialOrientation
        initialVelocity
        elasticityCoeff
        staticFrictionCoeff
        kineticFrictionCoeff
        radius
        mass
        position
        =
        let spehere = { Radius = radius }

        create
            initialOrientation
            initialVelocity
            elasticityCoeff
            staticFrictionCoeff
            kineticFrictionCoeff
            mass
            position
            (spehere.CreateRotationalInertia(mass.GetValue))

    let createDefaultSphere = createSphere RotationMatrix3D.zero Vector3D.zero

    let toWorldCoordinates (rigidBody: RigidBody) offset =
        offset
        |> GraphicsUtils.toWorldCoordinates rigidBody.Variables.Orientation rigidBody.MassCenterPosition

module internal RigidBodyIntegrators =
    open Vector3D
    open Matrix3

    let (firstOrder: RigidBodyIntegrator) =
        fun dt (torque: Torque) inertiaInverse old ->
            let totalSeconds = dt.TotalSeconds

            let angMomentumChange = torque * totalSeconds

            let newAngMomentum = old.AngularMomentum + angMomentumChange

            let angularVelocity = inertiaInverse * newAngMomentum

            let axis = angularVelocity * totalSeconds
            let angle = axis |> l2Norm

            let newOrientation =
                (angle |> RotationMatrix3D.fromAxisAndAngle (axis |> normalized))
                * old.Orientation

            { Orientation = newOrientation |> orthonormalize
              AngularMomentum = newAngMomentum }

    let (augmentedSecondOrder: RigidBodyIntegrator) =
        fun dt torque inertiaInverse old ->
            let totalSeconds = dt.TotalSeconds

            let angMomentumChange = torque * totalSeconds

            let oldAngMomentum = old.AngularMomentum

            let angularVelocity = inertiaInverse * oldAngMomentum

            let v = oldAngMomentum |> crossProduct angularVelocity

            let deriv = inertiaInverse * (torque - v)

            let comp1 = angularVelocity |> crossProduct deriv

            let axisAverage =
                angularVelocity
                + totalSeconds * 0.5 * deriv
                + (totalSeconds * totalSeconds / 12.0) * comp1

            let axis = axisAverage * totalSeconds
            let angle = axis |> l2Norm

            let newOrientation =
                (RotationMatrix3D.fromAxisAndAngle (axis |> normalized) angle) * old.Orientation

            { Orientation = newOrientation |> orthonormalize
              AngularMomentum = old.AngularMomentum + angMomentumChange }

module internal RigidBodyMotion =

    let applyImpulse (offset: Vector3D) rigidBody impulse =
        { rigidBody with
            MassCenter = impulse |> ParticleMotion.applyImpulse rigidBody.MassCenter
            Variables.AngularMomentum = rigidBody.Variables.AngularMomentum + (impulse |> Vector3D.crossProduct offset) }

    let applyImpulses (impulses: Vector3D list) offset (rigidBody: RigidBody) =
        impulses |> List.fold (applyImpulse offset) rigidBody

    let update
        (particleIntegrator: ParticleIntegrator)
        (rigidBodyIntegrator: RigidBodyIntegrator)
        (totalForce: Vector3D)
        (totalTorque: Vector3D)
        dt
        (rigidBody: RigidBody)
        =
        let acceleration = totalForce / rigidBody.MassCenter.Mass.GetValue

        let newLinearComponent =
            particleIntegrator dt acceleration rigidBody.MassCenter.Variables

        let newRotComponent =
            rigidBodyIntegrator dt totalTorque (rigidBody.CalcRotationalInertiaInverse()) rigidBody.Variables

        { rigidBody with
            Variables = newRotComponent
            MassCenter.Variables = newLinearComponent }

    let calculateTranslationConstraintMass bodies offsets =
        let K body (offset: Vector3D) =
            match body.MassCenter.Mass with
            | Mass.Infinite -> Matrix3.zero
            | Mass.Value _ ->
                body.MassCenter.GetInverseMassMatrix()
                + (offset |> Matrix3.hatOperator |> Matrix3.transposed)
                  * body.CalcRotationalInertiaInverse()
                  * (offset |> Matrix3.hatOperator)

        (bodies, offsets)
        ||> SetOf2.zip
        |> SetOf2.fold (fun s (body, offset) -> s + K body offset) Matrix3.zero

    let calculateVelocityAtOffset (body: RigidBody) offset =
        let getLinearVelocityAtOffset (rigidBody: RigidBody) =
            offset |> Vector3D.crossProduct (rigidBody.CalcAngularVelocity())

        let linearV = body.MassCenterVelocity
        let angularComponent = getLinearVelocityAtOffset body
        linearV + angularComponent

    /// calculates relative velocity between bodies at offsets.
    /// Returned velocity points from first body to second body 
    let calculateRelativeVelocity bodies offsets =
        (offsets |> SetOf2.snd |> calculateVelocityAtOffset (bodies |> SetOf2.snd))
        - (offsets |> SetOf2.fst |> calculateVelocityAtOffset (bodies |> SetOf2.fst))
