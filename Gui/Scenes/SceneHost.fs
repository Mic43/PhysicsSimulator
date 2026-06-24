namespace Gui.Scenes

open FSharp.Data.Adaptive
open PhysicsSimulator
open PhysicsSimulator.Collisions
open PhysicsSimulator.Entities

/// Bridges a physics scene to adaptive lists consumed by the renderer.
type SceneHost(scene: IPhysicsScene) =
    let collisions = ChangeableIndexList<CollisionData> []
    let objects = ChangeableIndexList<SimulatorObject> []

    member _.Scene = scene
    member _.Collisions = collisions :> alist<_>
    member _.Objects = objects :> alist<_>

    member this.Connect() =
        let simulator = scene.GetSimulator()

        simulator.SimulationStateChanged.Add(fun _ ->
            transact (fun () ->
                collisions.UpdateTo(simulator.AllCollisions) |> ignore
                objects.UpdateTo simulator.AllPhysicalObjects |> ignore)
            |> ignore)

        transact (fun () ->
            collisions.UpdateTo(simulator.AllCollisions) |> ignore
            objects.UpdateTo simulator.AllPhysicalObjects |> ignore)
