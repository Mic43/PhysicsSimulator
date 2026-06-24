namespace Gui.Scenes

open Aardvark.Application
open PhysicsSimulator
open PhysicsSimulator.Entities

[<AbstractClass>]
type SceneBase(simulator: Simulator) =
    member val Simulator = simulator

    interface IPhysicsScene with
        member this.GetSimulator() = this.Simulator
        member this.GroundObjectId = this.GroundObjectId
        member this.OnKeyDown key = this.OnKeyDown key

    abstract GroundObjectId : SimulatorObjectIdentifier
    abstract OnKeyDown : Keys -> unit
    default _.OnKeyDown _ = ()
