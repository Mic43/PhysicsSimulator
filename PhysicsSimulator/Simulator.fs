namespace PhysicsSimulator

open System
open FSharpPlus

type PhysicalObjectIdentifier =
    private
    | Value of int
    member this.Get() =
        match this with
        | Value i -> i

    static member fromInt i =
        if i < 0 then
            invalidArg "i" "must be positive"

        i |> Value

type PhysicalObject =
    | Particle of Particle
    | RigidBody of RigidBody

//type Box = unit
type Collider = Box of unit

type SimulatorObject =
    { PhysicalObject: PhysicalObject
      Collider: Collider }

type Simulator =
    private
        { Objects: Map<PhysicalObjectIdentifier, SimulatorObject> }
    member this.GetObjects() = this.Objects

module SimulatorObject =
    let createDefaultSphere radius position =
        { PhysicalObject =
            (RigidBody.createDefaultSphere 1.0 1.0 radius position)
            |> RigidBody
          Collider = () |> Box }

module Simulator =
    let private particleIntegrator =
        ParticleIntegrators.forwardEuler

    let private rigidBodyIntegrator =
        RigidBodyIntegrators.firstOrder

    let private updateObject dt =
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

    let fromObjects (objects: SimulatorObject seq) =
        { Objects =
            objects
            |> Seq.mapi (fun i o -> (i |> PhysicalObjectIdentifier.fromInt, o))
            |> Map.ofSeq }

    let update simulator (dt: TimeSpan) : Simulator =
        { simulator with
            Objects =
                simulator.Objects
                |> Map.mapValues (fun simObj ->
                    { simObj with PhysicalObject = simObj.PhysicalObject |> updateObject dt }) }
