namespace PhysicsSimulator.Entities

open FSharpPlus.Data
open PhysicsSimulator.Utilities
open FSharpPlus

type SimulatorObjectIdentifier =
    private
    | Value of int

    member this.Get =
        match this with
        | Value i -> i

    static member fromInt i =
        if i < 0 then
            invalidArg "i" "must be positive"

        i |> Value

    static member op_Implicit(i) = i |> SimulatorObjectIdentifier.fromInt

type PhysicalObject =
    | Particle of Particle
    | RigidBody of RigidBody

    member this.AsParticle() =
        match this with
        | Particle p -> p
        | RigidBody rigidBody -> rigidBody.MassCenter

    member this.MassCenterPosition() = this.AsParticle().Variables.Position


type SimulatorObject =
    { Id: SimulatorObjectIdentifier
      PhysicalObject: PhysicalObject
      Collider: Collider }

module SimulatorObject =

    // let createDefaultSphere id radius mass position =
    //     { PhysicalObject =
    //         (RigidBody.createDefaultSphere defaultElasticityCoeff defaultFrictionCoeff radius mass position)
    //         |> RigidBody
    //       Collider = radius |> Collider.createSphere
    //       Id = id }
    //
    // let createDefaultCube id size mass position =
    //     { PhysicalObject =
    //         (RigidBody.createDefaultBox defaultElasticityCoeff defaultFrictionCoeff size size size mass position)
    //         |> RigidBody
    //       Collider = (size, size, size) |||> Collider.createBox
    //       Id = id }

    let getOffsetFrom (point: Vector3D) (physicalObject: SimulatorObject) =
        (point, physicalObject.PhysicalObject.MassCenterPosition())
        ||> Vector3D.apply2 (-)

    let applyImpulse impulse offset object =
        let applyImpulseToObject object =
            match object with
            | RigidBody rigidBody -> RigidBodyMotion.applyImpulse rigidBody impulse offset |> RigidBody
            | Particle particle -> impulse |> ParticleMotion.applyImpulse particle |> Particle

        { object with
            PhysicalObject = object.PhysicalObject |> applyImpulseToObject }

    let update particleIntegrator rigidBodyIntegrator dt (totalForce: Vector3D) totalTorque =
        function
        | Particle p ->
            let acceleration = totalForce.Get / p.Mass

            let particleIntegrator = particleIntegrator dt (acceleration |> Vector3D.ofVector)

            { p with
                Variables = p.Variables |> particleIntegrator }
            |> Particle
        | RigidBody rigidBody ->
            let updater =
                RigidBodyMotion.update particleIntegrator rigidBodyIntegrator totalForce totalTorque dt

            let newState = rigidBody |> updater
            newState |> RigidBody
