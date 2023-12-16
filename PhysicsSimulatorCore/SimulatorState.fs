namespace PhysicsSimulator

open System
open FSharpPlus

type SimulatorState =
    private
        { Objects: Map<SimulatorObjectIdentifier, SimulatorObject>
          TotalForce: Map<SimulatorObjectIdentifier, Vector3D>
          TotalTorque: Map<SimulatorObjectIdentifier, Vector3D> }

    member this.GetObjects() = this.Objects

module SimulatorState =
    let private particleIntegrator = ParticleIntegrators.forwardEuler
    let private rigidBodyIntegrator = RigidBodyIntegrators.firstOrder
    let private earthGAcceleration = Vector3D.create 0.0 0.0 -9.81

    let private gravityForce (objects: Map<SimulatorObjectIdentifier, SimulatorObject>) identifier =
        objects.[identifier].PhysicalObject.AsParticle().Mass * earthGAcceleration.Get()
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
            let acceleration = totalForce.Get() / p.Mass

            let particleIntegrator = particleIntegrator dt (acceleration |> Vector3D.ofVector)

            { p with
                Variables = p.Variables |> particleIntegrator }
            |> Particle
        | RigidBody rigidBody ->
            let updater =
                RigidBodyMotion.update particleIntegrator rigidBodyIntegrator totalForce totalTorque dt

            rigidBody |> updater |> RigidBody

    let private updateObjectById simulatorState dt id =
        updateObject
            dt
            simulatorState.TotalForce[id]
            simulatorState.TotalTorque[id]
            simulatorState.Objects[id].PhysicalObject

    let fromObjects (objects: SimulatorObject seq) =
        let identifiers =
            objects |> Seq.mapi (fun i _ -> (i |> SimulatorObjectIdentifier.fromInt))

        let objectMap: Map<SimulatorObjectIdentifier, SimulatorObject> =
            objects |> Seq.zip identifiers |> Map.ofSeq

        { Objects = objectMap
          TotalForce =
            identifiers
            |> Seq.map (fun i -> (i, i |> calculateForce objectMap))
            |> Map.ofSeq
          TotalTorque = identifiers |> Seq.map (fun i -> (i, Vector3D.zero)) |> Map.ofSeq }

    let update (dt: TimeSpan) simulatorState : SimulatorState =
        { simulatorState with
            Objects =
                simulatorState.Objects
                |> Map.map (fun ident simObj ->
                    { simObj with
                        PhysicalObject =
                            simObj.PhysicalObject
                            |> updateObject dt simulatorState.TotalForce[ident] simulatorState.TotalTorque[ident] }) }

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
        |> Option.bind (fun cd ->
            "Collision detected " |> printfn "%A"
            cd |> printfn "%A"
          
            let cd2 = { cd with Normal = cd.Normal |> Vector3D.apply (~-) }

            let physicalObject1 = obj1NextState.PhysicalObject
            let physicalObject2 = obj2NextState.PhysicalObject

            let impulse1 = CollisionResponse.calculateImpulse cd physicalObject1 physicalObject2
            let offset1 = cd.ContactPoint.Get() - physicalObject1.MassCenterPosition().Get()

            let impulse2 = CollisionResponse.calculateImpulse cd2 physicalObject2 physicalObject1
            let offset2 = cd.ContactPoint.Get() - physicalObject2.MassCenterPosition().Get()

            let nextSimulationState =
                curSimulationState
                |> applyImpulse id1 impulse1 offset1
                |> applyImpulse id2 impulse2 offset2
            
            printfn "Applying impulses:"
            printfn $"Object1: Id {id1}: %A{impulse1} at offset %A{offset1}"
            printfn $"Object1: Id {id2}: %A{impulse2} at offset %A{offset2}"
              
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
