namespace Gui.Scenes

open Aardvark.Application
open FSharp.Data.Adaptive
open PhysicsSimulator
open PhysicsSimulator.Collisions
open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities

module private StackSceneData =
    let mass = 100.0

    let createVerticalStack position height boxSize massValue =
        [ 0 .. height - 1 ]
        |> List.map (fun i ->
            { (boxSize |> RigidBodyPrototype.createDefaultCube) with
                ElasticityCoeff = 0.1
                Mass = massValue |> Mass.Value
                Position = position + Vector3D.create 0 0 (i |> float) * (boxSize + 0.01) })

    let prototypes =
        [ { ((15.0, 15.0, 1.01) |||> RigidBodyPrototype.createDefaultBox) with
              Mass = Mass.Infinite
              UseGravity = false
              Pitch = 0.0
              Position = Vector3D.create 0 0 -0.5 } ]
        @ createVerticalStack ((0.0, 0.0, 1.5) |||> Vector3D.create) 3 1.0 mass
        @ [ { (0.5 |> RigidBodyPrototype.createDefaultCube) with
                Mass = 50.0 |> Mass.Value
                UseGravity = true
                Position = Vector3D.create 0 -5 0 } ]

    let configuration =
        { Configuration.getDefault with
            SimulationSpeedMultiplier = 1.0
            BroadPhaseCollisionDetectionKind =
                { LeafCapacity = 2
                  SpaceBoundaries =
                    { Min = (-15.0, -15.0, -15.0) |||> Vector3D.create
                      Max = (15.0, 15.0, 15.0) |||> Vector3D.create }
                  MaxDepth = 10 }
                |> BroadPhaseCollisionDetectionKind.SpatialTree
            StepConfig.GravityDirection =
                (0.0, 0.0, -1.0) |||> Vector3D.create |> NormalVector.create 0.01 }

/// Mirror of the former Program.fs scene setup.
type StackScene() =
    inherit SceneBase(StackSceneData.prototypes, StackSceneData.configuration)

    // Prototype order: ground (0), stack cubes (1–3), loose cube (4).
    static let groundObjectId = 0 |> SimulatorObjectIdentifier.fromInt
    static let impulsedObjectId = 4 |> SimulatorObjectIdentifier.fromInt

    static let impulseStrenght = 200.0
    static let impulseDir = Vector3D.create 0.0 1.0 0.
    static let impulseValue = (impulseDir |> Vector3D.normalized).Get * impulseStrenght
    static let impulseOffset = Vector3D.create 0.0 0.0 0.0

    override _.GroundObjectId = groundObjectId

    override _.HelpLines =
        [ "Scena: stos"
          "Space - impuls"
          "Enter - dodaj kostke" ]

    override this.OnKeyDown key =
        let simulator = this.Simulator

        match key with
        | Keys.Space ->
            transact (fun () ->
                simulator.ApplyImpulse impulsedObjectId impulseValue impulseOffset)
        | Keys.Enter ->
            transact (fun () ->
                { (0.5 |> RigidBodyPrototype.createDefaultCube) with
                    Mass = 50.0 |> Mass.Value
                    Position = Vector3D.create 0 -3 3 }
                |> simulator.AddObject)
        | _ -> ()
