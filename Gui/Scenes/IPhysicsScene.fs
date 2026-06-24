namespace Gui.Scenes

open Aardvark.Application
open PhysicsSimulator
open PhysicsSimulator.Entities

type IPhysicsScene =
    abstract member GetSimulator : unit -> Simulator
    abstract member GroundObjectId : SimulatorObjectIdentifier
    abstract member Reset : unit -> unit
    abstract member HelpLines : string list
    abstract member OnKeyDown : Keys -> unit
