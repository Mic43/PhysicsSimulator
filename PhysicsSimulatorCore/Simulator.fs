namespace PhysicsSimulator

open System
open System.Threading
open FSharpPlus
open Microsoft.FSharp.Control

type SimulatorTaskState =
    | Paused
    | Started
    | Stopped

type Simulator(simulatorObjects, ?simulationSpeedMultiplier0, ?configuration0) =
    let simulationSpeedMultiplier = defaultArg simulationSpeedMultiplier0 1.0
    let configuration = defaultArg configuration0 Configuration.getDefault
    let simulationStepInterval = TimeSpan.FromMilliseconds(10.0)
    let mutable taskState = SimulatorTaskState.Stopped
    let simulatorStateChanged = Event<SimulatorState>()
    let gate = new SemaphoreSlim(1)

    let simulatorState: SimulatorState ref =
        simulatorObjects
        |> SimulatorStateBuilder.fromPrototypes
        |> SimulatorStateBuilder.withConfiguration configuration
        |> ref

    let broadPhaseCollisions (simulatorState: SimulatorState) =
        (simulatorState.Objects |> Map.keys |> Set.ofSeq)

    let updateSimulation =
        async {
            let collidingObjectsCandidates = simulatorState.Value |> broadPhaseCollisions
            gate.Wait()
            let newState =
                simulatorState.Value
                |> CollisionResolver.resolveAll simulationStepInterval collidingObjectsCandidates
                |> JointsRestorer.restoreAll simulationStepInterval 
                |> SimulatorState.update simulationStepInterval

            simulatorState.Value <- newState
            gate.Release() |> ignore
            newState |> simulatorStateChanged.Trigger
        }

    let cancellationTokenSource = new CancellationTokenSource()

    interface IDisposable with
        member _.Dispose() = gate.Dispose()

    member this.SimulationStateChanged = simulatorStateChanged.Publish
    member this.State = taskState
    member this.ObjectsIdentifiers = simulatorState.Value.Objects.Keys |> set
    member this.CollisionsIdentifiers = simulatorState.Value.Collisions.Keys |> List.ofSeq

    member this.SimulatorObject
        with get identifier = simulatorState.Value.Objects[identifier]

    member this.PhysicalObject
        with get identifier = this.SimulatorObject(identifier).PhysicalObject

    member this.AllPhysicalObjects =
        simulatorState.Value.Objects |> Map.values |> List.ofSeq

    member this.Collision
        with get identifier = simulatorState.Value.Collisions |> Map.tryFind identifier

    member this.AllCollisions = simulatorState.Value.Collisions |> Map.values |> List.ofSeq

    member this.Configuration = simulatorState.Value.Configuration

    member this.AddObject object =
        gate.Wait()        
        simulatorState.Value <- simulatorState.Value |> SimulatorStateBuilder.withPrototype object
        gate.Release() |> ignore
        
        simulatorState.Value |> simulatorStateChanged.Trigger        
    member this.ApplyImpulse physicalObjectIdentifier impulseValue impulseOffset =
        gate.Wait()
        simulatorState.Value <-
            simulatorState.Value
            |> SimulatorState.applyImpulse physicalObjectIdentifier impulseValue impulseOffset
        gate.Release() |> ignore
        simulatorState.Value |> simulatorStateChanged.Trigger

    member this.Start() =
        if taskState <> SimulatorTaskState.Stopped then
            invalidOp "simulator should be in stopped state"

        taskState <- Started

        let rec updateSingleFrame () =
            async {
                do!
                    if taskState = Started then
                        updateSimulation
                    else
                        async { () }

                do! simulationStepInterval.Divide(simulationSpeedMultiplier) |> Async.Sleep

                return! updateSingleFrame ()
            }

        Async.Start(updateSingleFrame (), cancellationTokenSource.Token)

    member this.Stop() =
        if taskState = SimulatorTaskState.Stopped then
            invalidOp "simulator is already stopped"

        cancellationTokenSource.Cancel()

    member this.PauseResume() =
        match taskState with
        | Paused -> taskState <- Started
        | Started -> taskState <- Paused
        | Stopped -> invalidOp "simulator must be started or paused"
