namespace PhysicsSimulator

open System
open System.Threading
open System.Threading.Tasks
open FSharpPlus

type Simulator(simulatorObjects) =

    let objectsLocker = Object()
    let mutable updateTask: Task = null

    let simulatorData: SimulatorState ref =
        simulatorObjects |> SimulatorState.fromObjects |> ref

    member this.ObjectIdentifiers = simulatorData.Value.Objects.Keys |> set

    member this.PhysicalObject
        with get identifier =
            (fun _ -> simulatorData.Value.Objects.[identifier].PhysicalObject)
            |> lock objectsLocker

    member this.ApplyImpulse physicalObjectIdentifier impulseValue impulseOffset =
        (fun _ ->
            simulatorData.Value <-
                SimulatorState.applyImpulse simulatorData.Value physicalObjectIdentifier impulseValue impulseOffset)
        |> lock objectsLocker

    member private this.UpdateTask() =
        let interval = TimeSpan.FromMilliseconds(30.0)
        //let update = (SimulatorData.update simulatorData)

        while true do
            let newData = interval |> SimulatorState.update simulatorData.Value
            (fun _ -> simulatorData.Value <- newData) |> lock objectsLocker
            //TODO: remove
            Thread.Sleep(interval)
        ()

    member this.StartUpdateThread() =
        updateTask <- Task.Run(this.UpdateTask)
        ()

    member this.PauseUpdateThread() = ()
