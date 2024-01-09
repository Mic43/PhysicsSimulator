namespace PhysicsSimulator

open System
open FSharpPlus
open Utils

[<CustomEquality; NoComparison>]
type ValueSupplier<'T> when 'T: equality =
    | Zero
    | Constant of 'T
    | Variable of (unit -> 'T)

    override this.Equals(other) =
        match other with
        | :? ValueSupplier<'T> as other ->
            match (this, other) with
            | Zero, Zero -> true
            | Constant foo, Constant foo1 -> foo = foo1
            | _ -> false
        | _ -> false

type SimulatorState =
    private
        { Objects: Map<SimulatorObjectIdentifier, SimulatorObject>
          ExternalForces: Map<SimulatorObjectIdentifier, Vector3D ValueSupplier>
          ExternalTorque: Map<SimulatorObjectIdentifier, Vector3D> }

    member this.GetObjects = this.Objects
    member this.GetExternalForces = this.ExternalForces
    member this.GetExternalTorque = this.ExternalTorque


    member this.CalculateTotalForce objId =
        let valueSupplier = this.GetExternalForces[objId]

        match valueSupplier with
        | Zero -> Vector3D.zero
        | Constant vector3D -> vector3D
        | Variable unitFunc -> unitFunc ()

module SimulatorState =
    let private particleIntegrator = ParticleIntegrators.forwardEuler
    let private rigidBodyIntegrator = RigidBodyIntegrators.firstOrder
    let private earthGAcceleration = Vector3D.create 0.0 0.0 -9.81

    let private gravityForce (objects: Map<SimulatorObjectIdentifier, SimulatorObject>) identifier =
        objects.[identifier].PhysicalObject.AsParticle().Mass * earthGAcceleration.Get
        |> Vector3D.ofVector

    let private calculateForce (objects: Map<SimulatorObjectIdentifier, SimulatorObject>) identifier =
        // gravityForce objects identifier
        Vector3D.zero

    let private updateObject =
        SimulatorObject.update particleIntegrator rigidBodyIntegrator

    let private updateObjectById (simulatorState: SimulatorState) dt id =
        updateObject
            dt
            (id |> simulatorState.CalculateTotalForce)
            simulatorState.ExternalTorque[id]
            simulatorState.Objects[id].PhysicalObject

    let fromObjects (objects: SimulatorObject seq) =
        let identifiers =
            objects |> Seq.mapi (fun i _ -> (i |> SimulatorObjectIdentifier.fromInt))

        let objectMap: Map<SimulatorObjectIdentifier, SimulatorObject> =
            objects |> Seq.zip identifiers |> Map.ofSeq

        { Objects = objectMap
          ExternalForces =
            identifiers
            |> Seq.map (fun i -> (i, i |> calculateForce objectMap |> ValueSupplier.Constant))
            |> Map.ofSeq
          ExternalTorque = identifiers |> Seq.map (fun i -> (i, Vector3D.zero)) |> Map.ofSeq }

    let update (dt: TimeSpan) simulatorState : SimulatorState =
        { simulatorState with
            Objects =
                simulatorState.Objects
                |> Map.map (fun ident simObj ->
                    { simObj with
                        PhysicalObject =
                            simObj.PhysicalObject
                            |> updateObject
                                dt
                                (ident |> simulatorState.CalculateTotalForce)
                                simulatorState.ExternalTorque[ident] }) }

    let private changeSimulatorObject objectIdentifier newPhysicalOBject (simulationState: SimulatorState) =
        { simulationState with
            Objects =
                simulationState.Objects.Change(
                    objectIdentifier,
                    (fun simObj ->
                        if simObj.IsNone then
                            invalidArg (nameof objectIdentifier) "object doest exist in map"

                        newPhysicalOBject |> Some)
                ) }

    let applyImpulse objectIdentifier impulse (offset: Vector3D) simulatorState =
        let applyImpulseToObject = SimulatorObject.applyImpulse impulse offset

        let simulatorObject = simulatorState.Objects[objectIdentifier]

        simulatorState
        |> changeSimulatorObject objectIdentifier (simulatorObject |> applyImpulseToObject)

    let private tryHandleCollision
        (nextStates: SetOf2<SimulatorObjectIdentifier * SimulatorObject>)
        (curSimulationState: SimulatorState)
        =
        nextStates
        |> SetOf2.map snd
        |> CollisionDetection.areColliding
        |> Option.bind (fun collisionData ->
            "Collision detected " |> printfn "%A"
            collisionData |> printfn "%A"

            let resolvedObjects =
                CollisionResponse.resolveCollision collisionData (nextStates |> SetOf2.map snd)

            //  printfn "State after collision: "
            //  printfn $"%A{nextSimulationState.Objects[id1].PhysicalObject}"
            //  printfn $"%A{nextSimulationState.Objects[id2].PhysicalObject}"

            curSimulationState
            |> changeSimulatorObject (nextStates |> SetOf2.fst |> fst) (resolvedObjects |> SetOf2.fst)
            |> changeSimulatorObject (nextStates |> SetOf2.snd |> fst) (resolvedObjects |> SetOf2.snd)
            |> Some)

    let private withCollisionResponse dt (curSimulationState: SimulatorState) ids =

        let nextStates: SetOf2<SimulatorObjectIdentifier * SimulatorObject> =
            ids
            |> SetOf2.map (fun id ->
                (id,
                 { curSimulationState.Objects[id] with
                     PhysicalObject = id |> updateObjectById curSimulationState dt }))

        tryHandleCollision nextStates curSimulationState
        |> Option.defaultValue curSimulationState

    let withCollisionResponseGlobal
        dt
        (collidingObjectsCandidates: SimulatorObjectIdentifier Set)
        (curSimulationState: SimulatorState)
        =

        let withCollisionResponse simulatorState objectIdentifiers =
            objectIdentifiers |> SetOf2.ofSet |> withCollisionResponse dt simulatorState

        collidingObjectsCandidates
        |> subSetsOf2Tail
        |> Set.fold withCollisionResponse curSimulationState
