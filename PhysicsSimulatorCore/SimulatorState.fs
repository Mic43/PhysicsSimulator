namespace PhysicsSimulator

open System
open FSharpPlus
open Utils

type ValueSupplier<'T> =
    | Zero
    | Constant of 'T
    | Variable of (unit -> 'T)

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

    let private tryHandleCollision (obj1NextState, obj2NextState) (id1, id2) (curSimulationState: SimulatorState) =
        (obj1NextState, obj2NextState)
        ||> CollisionDetection.areColliding
        |> Option.bind (fun collisionData ->
            "Collision detected " |> printfn "%A"
            collisionData |> printfn "%A"

            let resolvedObjects =
                CollisionResponse.resolveCollision collisionData obj1NextState obj2NextState

            //            printfn "State after collision: "
            //  printfn $"%A{nextSimulationState.Objects[id1].PhysicalObject}"
            //          printfn $"%A{nextSimulationState.Objects[id2].PhysicalObject}"

            curSimulationState
            |> changeSimulatorObject id1 (resolvedObjects |> fst)
            |> changeSimulatorObject id2 (resolvedObjects |> snd)
            |> Some)

    let private withCollisionResponse dt (curSimulationState: SimulatorState) (id1, id2) =
        let obj1NextState =
            { curSimulationState.Objects[id1] with
                PhysicalObject = id1 |> updateObjectById curSimulationState dt }

        let obj2NextState =
            { curSimulationState.Objects[id2] with
                PhysicalObject = id2 |> updateObjectById curSimulationState dt }

        tryHandleCollision (obj1NextState, obj2NextState) (id1, id2) curSimulationState
        |> Option.defaultValue curSimulationState

    let withCollisionResponseGlobal
        dt
        (collidingObjectsCandidates: SimulatorObjectIdentifier Set)
        (curSimulationState: SimulatorState)
        =
        let setOf2ToPair set =
            match (set |> Set.toList) with
            | [ v1; v2 ] -> (v1, v2)
            | _ -> invalidArg "set" "set must be set of two "

        let withCollisionResponse simulatorState objectIdentifiers =
            objectIdentifiers |> setOf2ToPair |> withCollisionResponse dt simulatorState

        collidingObjectsCandidates
        |> subSetsOf2Tail
        |> Set.fold withCollisionResponse curSimulationState
