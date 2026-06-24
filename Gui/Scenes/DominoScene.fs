namespace Gui.Scenes

open Aardvark.Application
open FSharp.Data.Adaptive
open PhysicsSimulator
open PhysicsSimulator.Collisions
open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities

/// Ground plus a row of upright dominoes. Space pushes the first domino into the chain.
type DominoScene() =
    inherit SceneBase(DominoScene.createSimulator ())

    // Prototype order: ground (0), dominoes (1..N).
    static let groundObjectId = 0 |> SimulatorObjectIdentifier.fromInt
    static let impulsedObjectId = 1 |> SimulatorObjectIdentifier.fromInt

    static let dominoCount = 15
    static let dominoThickness = 0.08
    static let dominoWidth = 0.35
    static let dominoHeight = 1.2
    // Row along world Y; dominoes face each other with their wide X×Z sides (±Y faces).
    static let dominoSpacing = dominoThickness + 0.4

    static let impulseStrength = 120.0
    static let impulseDir = Vector3D.create 0.0 1.0 0.0
    static let impulseValue = (impulseDir |> Vector3D.normalized).Get * impulseStrength
    static let impulseOffset = Vector3D.create 0.0 0.0 0.0

    static member private createDominoRow () =
        let halfHeight = dominoHeight / 2.0
        let rowOriginY = -(float (dominoCount - 1) * dominoSpacing) / 2.0

        [ 0 .. dominoCount - 1 ]
        |> List.map (fun i ->
            { ((dominoWidth, dominoThickness, dominoHeight)
               |||> RigidBodyPrototype.createDefaultBox) with
                Mass = 100.0 |> Mass.Value
                ElasticityCoeff = 0.05
                StaticFrictionCoeff = 0.7
                KineticFrictionCoeff = 0.55
                Position =
                    Vector3D.create 0.0 (rowOriginY + float i * dominoSpacing) halfHeight })

    static member private createSimulator () =
        let rigidBodyPrototypes =
            [ { ((12.0, 12.0, 1.01) |||> RigidBodyPrototype.createDefaultBox) with
                  Mass = Mass.Infinite
                  UseGravity = false
                  Position = Vector3D.create 0 0 -0.5 } ]
            @ DominoScene.createDominoRow ()

        new Simulator(
            rigidBodyPrototypes,
            { Configuration.getDefault with
                SimulationSpeedMultiplier = 1.0
                BroadPhaseCollisionDetectionKind = BroadPhaseCollisionDetectionKind.Dummy
                    // { LeafCapacity = 2
                    //   SpaceBoundaries =
                    //     {| Min = (-10.0, -10.0, -5.0) |||> Vector3D.create
                    //        Max = (10.0, 10.0, 10.0) |||> Vector3D.create |}
                    //   MaxDepth = 10 }
                    // |> BroadPhaseCollisionDetectionKind.SpatialTree
                StepConfig.GravityDirection =
                    (0.0, 0.0, -1.0) |||> Vector3D.create |> NormalVector.create 0.01 }
        )

    override _.GroundObjectId = groundObjectId

    override this.OnKeyDown key =
        let simulator = this.Simulator

        match key with
        | Keys.Space ->
            transact (fun () ->
                simulator.ApplyImpulse impulsedObjectId impulseValue impulseOffset)
        | Keys.Pause -> transact (fun () -> simulator.PauseResume ())
        | _ -> ()
