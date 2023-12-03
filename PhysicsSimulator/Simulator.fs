namespace PhysicsSimulator

open System
open System.Threading
open System.Threading.Tasks
open FSharpPlus


type Simulator(simulatorObjects) =
         
    let mutable updateTask:Task = null
    let simulatorData: SimulatorData ref =
        simulatorObjects
        |> SimulatorData.fromObjects
        |> ref    
    member this.SimulatorData
        with get() = simulatorData
        
    member private this.UpdateTask() =
           let interval = TimeSpan.FromMilliseconds(30.0)
           //let update = (SimulatorData.update simulatorData)
           
           while true do                
                let newData = interval |> SimulatorData.update simulatorData.Value
                simulatorData.Value  <- newData
                //TODO: remove
                Thread.Sleep(interval)
           ()
    member this.StartUpdateThread() =
        updateTask <- Task.Run(this.UpdateTask)         
        ()
    member this.PauseUpdateThread() =        
        ()
        
