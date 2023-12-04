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

type PhysicalObject =
    | Particle of Particle
    | RigidBody of RigidBody

//type Box = unit
type Collider = Box of unit

type SimulatorObject =
    { PhysicalObject: PhysicalObject
      Collider: Collider }

type SimulatorData =
    private
        { Objects: Map<PhysicalObjectIdentifier, SimulatorObject>
          TotalForce: Map<PhysicalObjectIdentifier, Vector3D>
          TotalTorque: Map<PhysicalObjectIdentifier, Vector3D> }
    member this.GetObjects() = this.Objects


module SimulatorObject =
    let createDefaultSphere radius mass position =
        { PhysicalObject =
            (RigidBody.createDefaultSphere 1.0 radius mass position)
            |> RigidBody
          Collider = () |> Box }

    let createDefaultCube size mass position =
        { PhysicalObject =
            (RigidBody.createDefaultBox 1.0 size size size mass position)
            |> RigidBody
          Collider = () |> Box }

module SimulatorData =
    let private particleIntegrator =
        ParticleIntegrators.forwardEuler

    let private rigidBodyIntegrator =
        RigidBodyIntegrators.firstOrder

    let private defaultForce =
        Vector3D.create 0.0 0.0 -0.000001

    let private applyImpulseToObject impulse offset object =
        (match object with
         | RigidBody rigidBody -> RigidBodyMotion.applyImpulse rigidBody impulse offset)
        |> RigidBody

    let private updateObject dt (totalForce: Vector3D) totalTorque =
        function
        | Particle p ->
            let acceleration = totalForce.Get() / p.Mass

            let particleIntegrator =
                particleIntegrator dt (acceleration |> Vector3D.fromVector)

            { p with ParticleVariables = p.ParticleVariables |> particleIntegrator }
            |> Particle
        | RigidBody rigidBody ->
            let updater =
                RigidBodyMotion.update particleIntegrator rigidBodyIntegrator totalForce totalTorque dt

            rigidBody |> updater |> RigidBody

    let fromObjects (objects: SimulatorObject seq) =
        let identifiers =
            objects
            |> Seq.mapi (fun i _ -> (i |> PhysicalObjectIdentifier.fromInt))

        { Objects = objects |> Seq.zip identifiers |> Map.ofSeq
          TotalForce =
            identifiers
            |> Seq.map (fun i -> (i, defaultForce))
            |> Map.ofSeq
          TotalTorque =
            identifiers
            |> Seq.map (fun i -> (i, Vector3D.zero))
            |> Map.ofSeq }

    let update simulatorData (dt: TimeSpan) : SimulatorData =
        { simulatorData with
            Objects =
                simulatorData.Objects
                |> Map.map (fun ident simObj ->
                    { simObj with
                        PhysicalObject =
                            simObj.PhysicalObject
                            |> updateObject dt simulatorData.TotalForce.[ident] simulatorData.TotalTorque.[ident] }) }

    let applyImpulse simulatorData objectIdentifier impulse (offset: Vector3D) =
        let applyImpulseToObject =
            applyImpulseToObject impulse offset

        { simulatorData with
            Objects =
                simulatorData.Objects.Change(
                    objectIdentifier,
                    fun simObj ->
                        simObj
                        |> Option.map (fun so -> { so with PhysicalObject = so.PhysicalObject |> applyImpulseToObject })
                ) }
