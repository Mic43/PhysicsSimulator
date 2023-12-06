namespace PhysicsSimulator

open System
open System.Threading
open System.Threading.Tasks
open FSharpPlus
open MathNet.Numerics.LinearAlgebra


type Simulator(simulatorObjects) =


    let objectsLocker = Object()
    let mutable updateTask: Task = null

    let simulatorState: SimulatorState ref =
        simulatorObjects |> SimulatorState.fromObjects |> ref

    member this.ObjectIdentifiers = simulatorState.Value.Objects.Keys |> set

    member this.PhysicalObject
        with get identifier =
            (fun _ -> simulatorState.Value.Objects.[identifier].PhysicalObject)
            |> lock objectsLocker

    member this.ApplyImpulse physicalObjectIdentifier impulseValue impulseOffset =
        (fun _ ->
            simulatorState.Value <-
                SimulatorState.applyImpulse simulatorState.Value physicalObjectIdentifier impulseValue impulseOffset)
        |> lock objectsLocker

    member private this.UpdateTask() =
        let interval = TimeSpan.FromMilliseconds(30.0)
        
        while true do
           // let simulatorObject() = simulatorState.Value.Objects[0 |> PhysicalObjectIdentifier.fromInt].PhysicalObject
            
            // match simulatorObject() with
            //     | RigidBody rb -> printfn $"Before %A{rb.RigidBodyVariables}"
            
            let newData = interval |> SimulatorState.update simulatorState.Value
            (fun _ -> simulatorState.Value <- newData) |> lock objectsLocker
            
            // match simulatorObject() with
            //     | RigidBody rb -> printfn $"After %A{rb.RigidBodyVariables}"
            //TODO: remove
            Thread.Sleep(interval)
        ()

    member this.StartUpdateThread() =
                   
        updateTask <- Task.Run(this.UpdateTask)
        ()

    member this.PauseUpdateThread() = ()
