namespace Gui.Scenes

open Aardvark.Application
open FSharp.Data.Adaptive
open PhysicsSimulator
open PhysicsSimulator.Entities

[<AbstractClass>]
type SceneBase(prototypes: RigidBodyPrototype list, configuration: Configuration) =
    let simulator = new Simulator(prototypes, configuration)

    member this.Simulator = simulator

    member this.Reset() =
        transact (fun () -> simulator.Reset(prototypes))

    interface IPhysicsScene with
        member this.GetSimulator() = this.Simulator
        member this.GroundObjectId = this.GroundObjectId
        member this.Reset() = this.Reset()
        member this.OnKeyDown key = this.OnKeyDown key

    abstract GroundObjectId : SimulatorObjectIdentifier
    abstract OnKeyDown : Keys -> unit
    default _.OnKeyDown _ = ()
