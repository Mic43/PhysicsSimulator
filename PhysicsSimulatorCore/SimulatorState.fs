namespace PhysicsSimulator

open System
open FSharpPlus
open FSharpPlus.Data
open PhysicsSimulator.Collisions
open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities

type SimulatorState =
    private
        { Objects: Map<SimulatorObjectIdentifier, SimulatorObject>
          ExternalForces: Map<SimulatorObjectIdentifier, Vector3D ValueSupplier>
          ExternalTorque: Map<SimulatorObjectIdentifier, Vector3D>
          Configuration: Configuration }

    member this.GetObjects = this.Objects
    member this.GetExternalForces = this.ExternalForces
    member this.GetExternalTorque = this.ExternalTorque    
    member this.GetConfiguration = this.Configuration

module SimulatorStateBuilder =
    let private earthGAcceleration = Vector3D.create 0.0 0.0 -9.81
    let private gravityForce mass = mass * earthGAcceleration

    let private getExternalForceSupplier proto =
        (match proto.UseGravity with
         | true -> proto.Mass.GetValue() |> gravityForce
         | false -> Vector3D.zero)
        |> ValueSupplier.Constant

    let fromPrototypes (prototypes: RigidBodyPrototype seq) =
        let prototypes = prototypes |> Seq.toList

        let identifiers =
            prototypes |> List.mapi (fun i _ -> (i |> SimulatorObjectIdentifier.fromInt))

        let objectMap: Map<SimulatorObjectIdentifier, SimulatorObject> =
            prototypes
            |> List.zip identifiers
            |> List.map (fun pair ->
                pair
                |> Tuple2.mapItem2 (fun proto -> pair |> fst |> RigidBodyPrototype.build proto))
            |> Map.ofList

        let externalForces =
            prototypes
            |> List.zip identifiers
            |> List.map (Tuple2.mapItem2 getExternalForceSupplier)
            |> Map.ofList

        { Objects = objectMap
          ExternalForces = externalForces
          ExternalTorque = identifiers |> Seq.map (fun i -> (i, Vector3D.zero)) |> Map.ofSeq
          Configuration = Configuration.getDefault }

module SimulatorState =
    let private particleIntegrator = ParticleIntegrators.forwardEuler
    let private rigidBodyIntegrator = RigidBodyIntegrators.firstOrder

    let private objectUpdater =
        SimulatorObject.update particleIntegrator rigidBodyIntegrator

    let private updateObjectById (simulatorState: SimulatorState) dt id =
        let newObjectState =
            objectUpdater
                dt
                (simulatorState.GetExternalForces[id].GetValue())
                simulatorState.ExternalTorque[id]
                simulatorState.Objects[id].PhysicalObject

        { simulatorState.Objects[id] with
            PhysicalObject = newObjectState }

    let private changeSimulatorObject objectIdentifier newPhysicalObject (simulationState: SimulatorState) =
        { simulationState with
            Objects =
                simulationState.Objects.Change(
                    objectIdentifier,
                    (fun simObj ->
                        if simObj.IsNone then
                            invalidArg (nameof objectIdentifier) "object doest exist in map"

                        newPhysicalObject |> Some)
                ) }

    let update (dt: TimeSpan) simulatorState : SimulatorState =
        { simulatorState with
            Objects =
                simulatorState.Objects
                |> Map.mapValues (fun simObj -> updateObjectById simulatorState dt simObj.Id) }

    let applyImpulse objectIdentifier impulse (offset: Vector3D) simulatorState =
        let applyImpulseToObject = SimulatorObject.applyImpulse impulse offset

        let simulatorObject = simulatorState.Objects[objectIdentifier]

        simulatorState
        |> changeSimulatorObject objectIdentifier (simulatorObject |> applyImpulseToObject)

    let private tryHandleCollision
        dt
        (nextStates: SetOf2<SimulatorObject>)
        (curSimulationState: SimulatorState)
        : SimulatorState option =
        let simulatorState =
            monad {
                let! collisionData = nextStates |> CollisionDetection.areColliding

                match collisionData with
                | None -> None
                | Some collisionData ->
                    $"Collision detected between target: {(nextStates |> SetOf2.fst).Id} and other: {(nextStates |> SetOf2.snd).Id}"
                    |> printfn "%A"
                    collisionData |> printfn "%A"

                    let! resolvedObjects = CollisionResponse.resolveCollision dt collisionData nextStates

                    curSimulationState
                    |> changeSimulatorObject (nextStates |> SetOf2.fst).Id (resolvedObjects |> SetOf2.fst)
                    |> changeSimulatorObject (nextStates |> SetOf2.snd).Id (resolvedObjects |> SetOf2.snd)
                    |> Some
            }

        curSimulationState.Configuration |> Reader.run simulatorState

    let private withCollisionResponse dt (curSimulationState: SimulatorState) ids =

        let nextStates: SetOf2<SimulatorObject> =
            ids |> SetOf2.map (updateObjectById curSimulationState dt)

        tryHandleCollision dt nextStates curSimulationState
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
