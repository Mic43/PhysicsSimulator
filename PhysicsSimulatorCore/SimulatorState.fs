namespace PhysicsSimulator

open System
open FSharpPlus

//TODO: make possible to use as key in dictionary
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

    static member op_Implicit(i) = i |> PhysicalObjectIdentifier.fromInt

type PhysicalObject =
    | Particle of Particle
    | RigidBody of RigidBody

    member this.AsParticle() =
        match this with
        | Particle p -> p
        | RigidBody rigidBody -> rigidBody.MassCenter


//type Box = unit
type Collider = Box of unit

type SimulatorObject =
    { PhysicalObject: PhysicalObject
      Collider: Collider }

type SimulatorState =
    private
        { Objects: Map<PhysicalObjectIdentifier, SimulatorObject>
          TotalForce: Map<PhysicalObjectIdentifier, Vector3D>
          TotalTorque: Map<PhysicalObjectIdentifier, Vector3D> }

    member this.GetObjects() = this.Objects


module SimulatorObject =
    let createDefaultSphere radius mass position =
        { PhysicalObject = (RigidBody.createDefaultSphere 1.0 radius mass position) |> RigidBody
          Collider = () |> Box }

    let createDefaultCube size mass position =
        { PhysicalObject = (RigidBody.createDefaultBox 1.0 size size size mass position) |> RigidBody
          Collider = () |> Box }

module SimulatorState =
    let private particleIntegrator = ParticleIntegrators.forwardEuler

    let private rigidBodyIntegrator = RigidBodyIntegrators.firstOrder

    let private earthGAcceleration = Vector3D.create 0.0 0.0 -9.81

    let private gravityForce (objects: Map<PhysicalObjectIdentifier, SimulatorObject>) identifier =
        objects.[identifier].PhysicalObject.AsParticle().Mass * earthGAcceleration.Get()
        |> Vector3D.fromVector

    let private calculateForce (objects: Map<PhysicalObjectIdentifier, SimulatorObject>) identifier =
        gravityForce objects identifier
    //Vector3D.zero

    let private applyImpulseToObject impulse offset object =
        match object with
        | RigidBody rigidBody -> RigidBodyMotion.applyImpulse rigidBody impulse offset |> RigidBody
        | Particle particle -> impulse |> ParticleMotion.applyImpulse particle |> Particle

    let private updateObject dt (totalForce: Vector3D) totalTorque =
        function
        | Particle p ->
            let acceleration = totalForce.Get() / p.Mass

            let particleIntegrator = particleIntegrator dt (acceleration |> Vector3D.fromVector)

            { p with
                Variables = p.Variables |> particleIntegrator }
            |> Particle
        | RigidBody rigidBody ->
            let updater =
                RigidBodyMotion.update particleIntegrator rigidBodyIntegrator totalForce totalTorque dt

            rigidBody |> updater |> RigidBody

    let fromObjects (objects: SimulatorObject seq) =
        let identifiers =
            objects |> Seq.mapi (fun i _ -> (i |> PhysicalObjectIdentifier.fromInt))

        let objectMap: Map<PhysicalObjectIdentifier, SimulatorObject> =
            objects |> Seq.zip identifiers |> Map.ofSeq

        { Objects = objectMap
          TotalForce =
            identifiers
            |> Seq.map (fun i -> (i, i |> calculateForce objectMap))
            |> Map.ofSeq
          TotalTorque = identifiers |> Seq.map (fun i -> (i, Vector3D.zero)) |> Map.ofSeq }

    let update simulatorData (dt: TimeSpan) : SimulatorState =
        { simulatorData with
            Objects =
                simulatorData.Objects
                |> Map.map (fun ident simObj ->
                    { simObj with
                        PhysicalObject =
                            simObj.PhysicalObject
                            |> updateObject dt simulatorData.TotalForce[ident] simulatorData.TotalTorque[ident] }) }

    let applyImpulse simulatorState objectIdentifier impulse (offset: Vector3D) =
        let applyImpulseToObject = applyImpulseToObject impulse offset

        { simulatorState with
            Objects =
                simulatorState.Objects.Change(
                    objectIdentifier,
                    fun simObj ->
                        simObj
                        |> Option.map (fun so ->
                            { so with
                                PhysicalObject = so.PhysicalObject |> applyImpulseToObject })
                ) }
