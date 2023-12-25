namespace PhysicsSimulator

open System
open FSharpPlus

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

    let private applyImpulseToObject impulse offset object =
        match object with
        | RigidBody rigidBody -> RigidBodyMotion.applyImpulse rigidBody impulse offset |> RigidBody
        | Particle particle -> impulse |> ParticleMotion.applyImpulse particle |> Particle

    let private updateObject dt (totalForce: Vector3D) totalTorque =
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

            rigidBody |> updater |> RigidBody

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

    let applyImpulse objectIdentifier impulse (offset: Vector3D) simulatorState =
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

    let private tryHandleCollision (obj1NextState, obj2NextState) (id1, id2) (curSimulationState: SimulatorState) =
        (obj1NextState, obj2NextState)
        ||> CollisionDetection.areColliding
        |> Option.bind (fun collisionData ->
            "Collision detected " |> printfn "%A"
            collisionData |> printfn "%A"

            let getOffsets (physicalObject: PhysicalObject) (collisionData: CollisionData) =
                let getOffset (physicalObject: PhysicalObject) (contactPoint: Vector3D) =
                    (contactPoint, physicalObject.MassCenterPosition()) ||> Vector3D.apply2 (-)

                collisionData.ContactPoints |> Seq.map (getOffset physicalObject)

            let collisionData2 =
                { collisionData with
                    Normal = collisionData.Normal |> Vector3D.apply (~-) }

            let physicalObject1 = obj1NextState.PhysicalObject
            let physicalObject2 = obj2NextState.PhysicalObject

            let impulsesOffsets1 =
                CollisionResponse.calculateImpulses collisionData physicalObject1 physicalObject2
                |> Seq.zip (collisionData |> getOffsets physicalObject1)
                |> Seq.map (fun (offset, impulse) -> {| Offset = offset; Impulse = impulse |})

            let impulsesOffsets2 =
                CollisionResponse.calculateImpulses collisionData2 physicalObject2 physicalObject1
                |> Seq.zip (collisionData2 |> getOffsets physicalObject2)
                |> Seq.map (fun (offset, impulse) -> {| Offset = offset; Impulse = impulse |})

            let applyImpulsesAtOffsets
                (impulsesOffsets:
                    {| Impulse: Vector3D
                       Offset: Vector3D |} seq)
                objId
                simState
                =
                impulsesOffsets
                |> Seq.fold (fun state io -> applyImpulse objId io.Impulse io.Offset state) simState

            let nextSimulationState =
                curSimulationState
                |> applyImpulsesAtOffsets impulsesOffsets1 id1
                |> applyImpulsesAtOffsets impulsesOffsets2 id2

            printfn "Applying impulses:"
            printfn $"Object1: Id {id1}: %A{impulsesOffsets1} "
            printfn $"Object1: Id {id2}: %A{impulsesOffsets2} "

            //            printfn "State after collision: "
            //  printfn $"%A{nextSimulationState.Objects[id1].PhysicalObject}"
            //          printfn $"%A{nextSimulationState.Objects[id2].PhysicalObject}"

            nextSimulationState |> Some)

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
        |> Utils.subSetsOf2Tail
        |> Set.fold withCollisionResponse curSimulationState
