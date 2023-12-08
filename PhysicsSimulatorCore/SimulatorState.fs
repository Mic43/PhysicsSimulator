namespace PhysicsSimulator

open System
open FSharpPlus

type SimulatorState =
    private
        { Objects: Map<PhysicalObjectIdentifier, SimulatorObject>
          TotalForce: Map<PhysicalObjectIdentifier, Vector3D>
          TotalTorque: Map<PhysicalObjectIdentifier, Vector3D> }

    member this.GetObjects() = this.Objects

module SimulatorState =
    let private particleIntegrator = ParticleIntegrators.forwardEuler
    let private rigidBodyIntegrator = RigidBodyIntegrators.firstOrder
    let private earthGAcceleration = Vector3D.create 0.0 0.0 -9.81

    let private gravityForce (objects: Map<PhysicalObjectIdentifier, SimulatorObject>) identifier =
        objects.[identifier].PhysicalObject.AsParticle().Mass * earthGAcceleration.Get()
        |> Vector3D.fromVector

    let private calculateForce (objects: Map<PhysicalObjectIdentifier, SimulatorObject>) identifier =
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

            let particleIntegrator = particleIntegrator dt (acceleration |> Vector3D.fromVector)

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
            objects |> Seq.mapi (fun i _ -> (i |> PhysicalObjectIdentifier.fromInt))

        let objectMap: Map<PhysicalObjectIdentifier, SimulatorObject> =
            objects |> Seq.zip identifiers |> Map.ofSeq

        { Objects = objectMap
          TotalForce =
            identifiers
            |> Seq.map (fun i -> (i, i |> calculateForce objectMap))
            |> Map.ofSeq
          TotalTorque = identifiers |> Seq.map (fun i -> (i, Vector3D.zero)) |> Map.ofSeq }

    let update simulatorState (dt: TimeSpan) : SimulatorState =
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


    let private handleCollision (obj1NextState, obj2NextState) (id1, id2) (curSimulationState: SimulatorState) =
        (obj1NextState, obj2NextState)
        ||> CollisionDetection.areColliding
        |> Option.bind (fun cd ->
            "Collision detected " |> printfn "%A"
            let calcImpulse = cd |> CollisionResponse.calculateImpulse

            let physicalObject1 = obj1NextState.PhysicalObject
            let physicalObject2 = obj2NextState.PhysicalObject

            let impulse1 = calcImpulse physicalObject1 physicalObject2
            let offset1 = (cd.ContactPoint.Get() - physicalObject1.MassCenterPosition().Get())

            let impulse2 = calcImpulse physicalObject2 physicalObject1
            let offset2 = (cd.ContactPoint.Get() - physicalObject2.MassCenterPosition().Get())

            curSimulationState
            |> applyImpulse id1 impulse1 offset1
            |> applyImpulse id2 impulse2 offset2
            |> Some)

    let withCollisionResponse (id1, id2) (curSimulationState: SimulatorState) dt =
        let obj1NextState =
            { curSimulationState.Objects[id1] with
                PhysicalObject = id1 |> updateObjectById curSimulationState dt }

        let obj2NextState =
            { curSimulationState.Objects[id2] with
                PhysicalObject = id2 |> updateObjectById curSimulationState dt }

        handleCollision (obj1NextState, obj2NextState) (id1, id2) curSimulationState
        |> Option.defaultValue curSimulationState
