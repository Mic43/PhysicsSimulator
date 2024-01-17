namespace PhysicsSimulator

open System
open System.Threading
open FSharpPlus
open Microsoft.FSharp.Control


type SimulatorTaskState =
    | Paused
    | Started
    | Stopped

type Simulator(simulatorObjects) =

    let simulatorState: SimulatorState ref =
        simulatorObjects |> SimulatorStateBuilder.fromPrototypes |> ref

    let simulationStepInterval = TimeSpan.FromMilliseconds(10.0)
    let simulationSpeedMultiplier = 1
    let mutable taskState = SimulatorTaskState.Stopped

    let broadPhaseCollisions (simulatorState: SimulatorState) =
        (simulatorState.GetObjects |> Map.keys |> Set.ofSeq)

    let updateSimulation =
        async {
            let collidingObjectsCandidates = simulatorState.Value |> broadPhaseCollisions

            let newState =
                simulatorState.Value
                |> SimulatorState.withCollisionResponseGlobal simulationStepInterval collidingObjectsCandidates
                |> SimulatorState.update simulationStepInterval

            //            if newState <> simulatorState.Value then
            //                newState |> printfn "%A"

            simulatorState.Value <- newState
        }

    let cancellationTokenSource = new CancellationTokenSource()

    member this.ObjectIdentifiers = simulatorState.Value.GetObjects.Keys |> set

    member this.SimulatorObject
        with get identifier = simulatorState.Value.GetObjects[identifier]

    member this.PhysicalObject
        with get identifier = this.SimulatorObject(identifier).PhysicalObject

    member this.ApplyImpulse physicalObjectIdentifier impulseValue impulseOffset =
        simulatorState.Value <-
            simulatorState.Value
            |> SimulatorState.applyImpulse physicalObjectIdentifier impulseValue impulseOffset

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
