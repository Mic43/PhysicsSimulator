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

    let simulationStepInterval = TimeSpan.FromMilliseconds(10.0)
    let simulationSpeedMultiplier = 1.0
   
    let broadPhaseCollisions (simulatorState:SimulatorState) = (simulatorState.GetObjects |> Map.keys |> Set.ofSeq) 
    let updateSimulation =
        async {
            let collidingObjectsCandidates =
                simulatorState.Value |> broadPhaseCollisions

            let newState =
                simulatorState.Value         
                |> SimulatorState.withCollisionResponseGlobal simulationStepInterval collidingObjectsCandidates
                |> SimulatorState.update simulationStepInterval

//            if newState <> simulatorState.Value then
//                newState |> printfn "%A"
                
            (fun _ -> simulatorState.Value <- newState) |> lock objectsLocker
        }

    let cancellationTokenSource = new CancellationTokenSource()

    member this.ObjectIdentifiers = simulatorState.Value.GetObjects.Keys |> set

    member this.SimulatorObject
        with get identifier = (fun _ -> simulatorState.Value.GetObjects[identifier]) |> lock objectsLocker

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
                do! simulationStepInterval.Divide(simulationSpeedMultiplier) |> Async.Sleep

                return! updateSingleFrame ()
            }
            
        Async.Start(updateSingleFrame (), cancellationTokenSource.Token)

    member this.PauseUpdateThread() = cancellationTokenSource.Cancel()
