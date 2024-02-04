namespace PhysicsSimulator

open System
open FSharpPlus
open FSharpPlus.Data
open Microsoft.FSharp.Collections
open PhysicsSimulator.Collisions
open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities

type SimulatorState =
    private
        { Objects: Map<SimulatorObjectIdentifier, SimulatorObject>
          ExternalForces: Map<SimulatorObjectIdentifier, Vector3D ValueSupplier>
          ExternalTorque: Map<SimulatorObjectIdentifier, Vector3D>
          Collisions: Map<SimulatorObjectIdentifier Set, CollisionData>
          Configuration: Configuration }

module SimulatorStateBuilder =
    let private earthGAcceleration = Vector3D.create 0.0 0.0 -9.81
    let private gravityForce mass = mass * earthGAcceleration

    let private getExternalForceSupplier proto =
        (match proto.UseGravity with
         | true -> proto.Mass.GetValue |> gravityForce
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
          Configuration = Configuration.getDefault
          Collisions = Map.empty }

    let withConfiguration configuration state =
        { state with
            Configuration = configuration }

module SimulatorState =
    let private particleIntegrator = ParticleIntegrators.forwardEuler
    let private rigidBodyIntegrator = RigidBodyIntegrators.firstOrder

    let private objectUpdater =
        SimulatorObject.update particleIntegrator rigidBodyIntegrator

    let internal updateObjectById (simulatorState: SimulatorState) dt id =
        let newObjectState =
            objectUpdater
                dt
                (simulatorState.ExternalForces[id].GetValue())
                simulatorState.ExternalTorque[id]
                simulatorState.Objects[id].PhysicalObject

        { simulatorState.Objects[id] with
            PhysicalObject = newObjectState }

    let internal changeSimulatorObject objectIdentifier newPhysicalObject (simulationState: SimulatorState) =
        { simulationState with
            Objects =
                simulationState.Objects.Change(
                    objectIdentifier,
                    (fun simObj ->
                        if simObj.IsNone then
                            invalidArg (nameof objectIdentifier) "object doest exist in map"

                        newPhysicalObject |> Some)
                ) }

    let internal changeSimulatorObjects simulatorObjects simulationState =
        simulatorObjects
        |> List.fold (fun state obj -> state |> changeSimulatorObject obj.Id obj) simulationState

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
