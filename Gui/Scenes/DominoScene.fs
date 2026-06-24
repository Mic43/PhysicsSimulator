namespace Gui.Scenes

open Aardvark.Application
open FSharp.Data.Adaptive
open PhysicsSimulator
open PhysicsSimulator.Collisions
open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities

module private DominoSceneData =
    let dominoCount = 15
    let dominoThickness = 0.08
    let dominoWidth = 0.35
    let dominoHeight = 1.2
    // Row along world Y; dominoes face each other with their wide X×Z sides (±Y faces).
    let dominoSpacing = dominoThickness + 0.4

    let createDominoRow () =
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

    let prototypes =
        [ { ((12.0, 12.0, 1.01) |||> RigidBodyPrototype.createDefaultBox) with
              Mass = Mass.Infinite
              UseGravity = false
              Position = Vector3D.create 0 0 -0.5 } ]
        @ createDominoRow ()

    let configuration =
        { Configuration.getDefault with
            SimulationSpeedMultiplier = 1.0
            BroadPhaseCollisionDetectionKind = BroadPhaseCollisionDetectionKind.Dummy
            StepConfig.GravityDirection =
                (0.0, 0.0, -1.0) |||> Vector3D.create |> NormalVector.create 0.01 }

/// Ground plus a row of upright dominoes. Space pushes the first domino into the chain.
type DominoScene() =
    inherit SceneBase(DominoSceneData.prototypes, DominoSceneData.configuration)

    // Prototype order: ground (0), dominoes (1..N).
    static let groundObjectId = 0 |> SimulatorObjectIdentifier.fromInt
    static let impulsedObjectId = 1 |> SimulatorObjectIdentifier.fromInt

    static let impulseStrength = 120.0
    static let impulseDir = Vector3D.create 0.0 1.0 0.0
    static let impulseValue = (impulseDir |> Vector3D.normalized).Get * impulseStrength
    static let impulseOffset = Vector3D.create 0.0 0.0 0.0

    override _.GroundObjectId = groundObjectId

    override _.HelpLines =
        [ "Scena: domino"
          "Space - impuls na pierwsze domino" ]

    override this.OnKeyDown key =
        let simulator = this.Simulator

        match key with
        | Keys.Space ->
            transact (fun () ->
                simulator.ApplyImpulse impulsedObjectId impulseValue impulseOffset)
        | _ -> ()
