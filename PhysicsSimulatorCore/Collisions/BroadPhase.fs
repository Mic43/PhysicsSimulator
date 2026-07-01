namespace PhysicsSimulator.Collisions

open FSharpPlus
open FSharpPlus.Data
open Microsoft.FSharp.Collections
open PhysicsSimulator
open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities
open PhysicsSimulator.Utilities.SpatialTree

type CollisionsCandidates = SimulatorObjectIdentifier SetOf2 List

type internal BroadPhaseCollisionDetectorData =
    | Dummy
    | SpatialTree of
        {| Tree: SimulatorObjectIdentifier SpatialTree
           Config: SpatialTreeConfiguration |}

module internal BroadPhase =

    let private objExtentProvider (simulationObjectsMap: Map<SimulatorObjectIdentifier, SimulatorObject>) id =
        let bb = simulationObjectsMap[id] |> SimulatorObject.getAABoundingBox

        let minPosition = bb.CenterPosition - (bb.Size |> Box.toVector3D) / 2.0

        [ { Size = bb.Size.XSize
            Position = minPosition.X }
          { Size = bb.Size.YSize
            Position = minPosition.Y }
          { Size = bb.Size.ZSize
            Position = minPosition.Z } ]

    let private dummy_init (simulationObjectsMap: Map<SimulatorObjectIdentifier, SimulatorObject>) =
        simulationObjectsMap
        |> Map.keys
        |> Set.ofSeq
        |> subSetsOf2Tail
        |> Set.toList
        |> List.map SetOf2.ofSet

    let private initSpatialTree
        (configuration: SpatialTreeConfiguration)
        (simulationObjectsMap: Map<SimulatorObjectIdentifier, SimulatorObject>)
        =

        let ids = simulationObjectsMap |> Map.keys |> Set.ofSeq

        let inserter tree id =
            id |> SpatialTree.insert tree (objExtentProvider simulationObjectsMap)

        let boundaries =
            (configuration.SpaceBoundaries.Min.Get.AsArray(), configuration.SpaceBoundaries.Max.Get.AsArray())
            ||> Array.zip
            |> Array.toList

        let leafCapacity = configuration.LeafCapacity
        let maxDepth = configuration.MaxDepth

        let initialTree = SpatialTree.init leafCapacity maxDepth boundaries
        ids |> Set.fold inserter initialTree

    let private getCollisionCandidates tree =
        tree
        |> SpatialTree.getObjectBuckets
        |> Seq.filter (fun bucket -> bucket |> Set.count > 1)
        |> Seq.distinct
        |> Seq.collect (subSetsOf2Tail >> Set.toSeq)
        |> Seq.map SetOf2.ofSet
        |> List.ofSeq

    let init
        (simulationObjects: Map<SimulatorObjectIdentifier, SimulatorObject>)
        (broadPhaseCollisionDetectionKind: BroadPhaseCollisionDetectionKind)
        : BroadPhaseCollisionDetectorData =


        match broadPhaseCollisionDetectionKind with
        | BroadPhaseCollisionDetectionKind.Dummy -> Dummy
        | BroadPhaseCollisionDetectionKind.SpatialTree config ->
            let tree = initSpatialTree config simulationObjects
            {| Tree = tree; Config = config |} |> SpatialTree


    let update
        (simulationObjects: Map<SimulatorObjectIdentifier, SimulatorObject>)
        : State<BroadPhaseCollisionDetectorData, CollisionsCandidates> =
        monad {
            let! data = State.get

            match data with
            | Dummy -> dummy_init simulationObjects
            | SpatialTree oldTree ->
                let dynamicIds =
                    simulationObjects
                    |> Map.filter (fun _ obj -> obj.PhysicalObject.IsStatic() |> not)
                    |> Map.keys

                let withRemovedDynamicNodes: SpatialTree<SimulatorObjectIdentifier> =
                    dynamicIds |> Seq.fold (fun tree id -> id |> remove tree) oldTree.Tree

                let newTree: SpatialTree<SimulatorObjectIdentifier> =
                    dynamicIds
                    |> Seq.fold
                        (fun tree id -> id |> insert tree (objExtentProvider simulationObjects))
                        withRemovedDynamicNodes

                do! {| oldTree with Tree = newTree |} |> SpatialTree |> State.put
                return newTree |> getCollisionCandidates
        }
