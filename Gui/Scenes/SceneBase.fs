namespace Gui.Scenes

open Aardvark.Application
open Aardvark.Base
open Aardvark.Rendering
open FSharp.Data.Adaptive
open PhysicsSimulator
open PhysicsSimulator.Entities

[<AbstractClass>]
type SceneBase(prototypes: RigidBodyPrototype list, configuration: Configuration) =
    let simulator = new Simulator(prototypes, configuration)

    member this.Simulator = simulator

    interface IPhysicsScene with
        member this.GetSimulator() = this.Simulator
        member this.GroundObjectId = this.GroundObjectId
        member this.HelpLines = this.HelpLines
        member this.InitialCameraView = this.InitialCameraView
        member this.OnKeyDown key = this.OnKeyDown key

    abstract GroundObjectId : SimulatorObjectIdentifier
    abstract HelpLines : string list
    default _.HelpLines = []

    abstract InitialCameraView : CameraView

    default _.InitialCameraView =
        CameraView.LookAt(V3d(0.0, -2.0, 2.0), V3d.Zero, V3d.OOI)

    abstract OnKeyDown : Keys -> unit
    default _.OnKeyDown _ = ()
