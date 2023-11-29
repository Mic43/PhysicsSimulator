namespace PhysicsSimulator

open System

type PhysicalObject =
    | Particle of Particle
    | RigidBody of RigidBody

type Box = unit
type Collider = Box of Box

type SimulatorObject =
    { PhysicalObject: PhysicalObject
      Collider: Collider }

type Simulator = { Objects: SimulatorObject seq }

module Simulator =
    let particleIntegrator =
        ParticleIntegrators.forwardEuler

    let rigidBodyIntegrator =
        RigidBodyIntegrators.firstOrder

    let updateObject dt =
        function
        | Particle p ->
            let acceleration = Vector3D.zero

            let particleIntegrator =
                particleIntegrator dt acceleration

            { p with ParticleVariables = p.ParticleVariables |> particleIntegrator }
            |> Particle
        | RigidBody rigidBody ->
            let totalForce = Vector3D.zero
            let totalTorque = Vector3D.zero

            let updater =
                RigidBodyMotion.update particleIntegrator rigidBodyIntegrator totalForce totalTorque dt

            rigidBody |> updater |> RigidBody

    let update simulator (dt: TimeSpan) : Simulator =
        { simulator with
            Objects =
                simulator.Objects
                |> Seq.map (fun simObj -> { simObj with PhysicalObject = simObj.PhysicalObject |> updateObject dt }) }
