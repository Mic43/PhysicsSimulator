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
          ExternalTorques: Map<SimulatorObjectIdentifier, Vector3D>
          Collisions: Map<SimulatorObjectIdentifier Set, CollisionData>
          Joints: Joint list
          Configuration: StepConfiguration }

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
          ExternalTorques = identifiers |> Seq.map (fun i -> (i, Vector3D.zero)) |> Map.ofSeq
          Configuration = StepConfiguration.getDefault
          Collisions = Map.empty
          Joints =
            [ Vector3D.create 0 -5 1.25
              |> Joint.createBallSocket ([ objectMap[1]; objectMap[2] ] |> SetOf2.ofList) ] }

    let withConfiguration configuration simulatorState =
        { simulatorState with
            Configuration = configuration }

    let withPrototype objectProto simulatorState =
        let id =
            (simulatorState.Objects |> Map.keys |> Seq.max).Get + 1
            |> SimulatorObjectIdentifier.fromInt

        let object = id |> RigidBodyPrototype.build objectProto

        { simulatorState with
            Objects = simulatorState.Objects |> Map.add id object
            ExternalForces =
                simulatorState.ExternalForces
                |> Map.add id (objectProto |> getExternalForceSupplier)
            ExternalTorques = simulatorState.ExternalTorques |> Map.add id Vector3D.zero }

module SimulatorState =
    let private particleIntegrator = ParticleIntegrators.forwardEuler
    let private rigidBodyIntegrator = RigidBodyIntegrators.firstOrder

    let internal updateObjectById (simulatorState: SimulatorState) dt id =
        let objectUpdater =
            SimulatorObject.update
                particleIntegrator
                (match simulatorState.Configuration.RigidBodyIntegratorKind with
                 | FirstOrder -> RigidBodyIntegrators.firstOrder
                 | AugmentedSecondOrder -> RigidBodyIntegrators.augmentedSecondOrder)

        let newObjectState =
            objectUpdater
                dt
                (simulatorState.ExternalForces[id].GetValue())
                simulatorState.ExternalTorques[id]
                simulatorState.Objects[id].PhysicalObject

        { simulatorState.Objects[id] with
            PhysicalObject = newObjectState }

    let internal changeSimulatorObject objectIdentifier simulatorObject (simulationState: SimulatorState) =
        { simulationState with
            Objects =
                simulationState.Objects.Change(
                    objectIdentifier,
                    (fun simObj ->
                        if simObj.IsNone then
                            invalidArg (nameof objectIdentifier) "object doest exist in map"

                        simulatorObject |> Some)
                ) }

    let internal changeSimulatorObjects simulatorObjects simulationState =
        simulatorObjects
        |> List.fold (fun state obj -> state |> changeSimulatorObject obj.Id obj) simulationState

    /// changes simulationObjects with physicalObjects.
    /// Physical objects are 'injected' into given simulation objects
    let internal changePhysicalObjects simulationObjectsIds physicalObjects simulationState =
        if (simulationObjectsIds |> List.length) <> (physicalObjects |> List.length) then
            invalidArg "simulationObjectsIds" "simulationObjectsIds and physicalObjects must have same len"

        let changed =
            (simulationState.Objects
             |> Map.filter (fun id value -> simulationObjectsIds |> List.contains id)
             |> Map.values
             |> List.ofSeq,
             physicalObjects)
            ||> List.zip
            |> List.map (fun p -> p ||> SimulatorObject.withPhysicalObject)

        simulationState |> changeSimulatorObjects changed

    let internal getJointObjects simulatorState joint =
        (simulatorState.Objects[joint.Identifiers |> SetOf2.fst],
         simulatorState.Objects[joint.Identifiers |> SetOf2.snd])
        |> SetOf2.ofPair

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
