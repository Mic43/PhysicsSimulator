namespace PhysicsSimulator

open System
open System.Threading
open System.Threading.Tasks
open FSharpPlus
open MathNet.Numerics.LinearAlgebra


type Simulator(simulatorObjects) =

    let objectsLocker = Object()

    let simulatorState: SimulatorState ref =
        simulatorObjects |> SimulatorState.fromObjects |> ref

    let interval = TimeSpan.FromMilliseconds(30.0)

    let updateSimulation =
        async {

            let tempState =
                interval |> SimulatorState.withCollisionResponse (0, 1) simulatorState.Value

            let newState = interval |> SimulatorState.update tempState
            newState |> printf "%A"
            (fun _ -> simulatorState.Value <- newState) |> lock objectsLocker
        }

    let cancellationTokenSource = new CancellationTokenSource()

    member this.ObjectIdentifiers = simulatorState.Value.Objects.Keys |> set

    member this.SimulatorObject
        with get identifier = (fun _ -> simulatorState.Value.Objects.[identifier]) |> lock objectsLocker

    member this.PhysicalObject
        with get identifier = this.SimulatorObject(identifier).PhysicalObject

    member this.ApplyImpulse physicalObjectIdentifier impulseValue impulseOffset =
        (fun _ ->
            simulatorState.Value <-
                simulatorState.Value
                |> SimulatorState.applyImpulse physicalObjectIdentifier impulseValue impulseOffset)
        |> lock objectsLocker

    //TODO: prevent multiple calls
    member this.StartUpdateThread() =
        let rec updateSingleFrame () =
            async {
                do! updateSimulation
                do! interval |> Async.Sleep

                return! updateSingleFrame ()
            }

        Async.Start(updateSingleFrame (), cancellationTokenSource.Token)

    member this.PauseUpdateThread() = cancellationTokenSource.Cancel()
