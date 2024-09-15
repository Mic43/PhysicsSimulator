namespace PhysicsSimulator.Collisions

open Microsoft.FSharp.Collections
open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities


type BroadPhaseCollisionDetector = SimulatorObjectIdentifier Set -> SimulatorObjectIdentifier SetOf2 List

type SpatialTreeConfiguration = { LeafCapacity: int; MaxDepth: int }

module BroadPhase =
    let dummy (ids: SimulatorObjectIdentifier Set) =
        ids |> subSetsOf2Tail |> Set.toList |> List.map SetOf2.ofSet

    let withSpatialTree
        (configuration:SpatialTreeConfiguration)
        (simulationObjectsMap: Map<SimulatorObjectIdentifier, SimulatorObject>)
        (spaceBoundaries: {| Min: Vector3D; Max: Vector3D |})
        (ids: SimulatorObjectIdentifier Set)
        =

        let objExtentProvider id =
            let bb = simulationObjectsMap[id] |> SimulatorObject.getAABoundingBox

            let minPosition = bb.CenterPosition - (bb.Size |> Box.toVector3D) / 2.0

            [ { Size = bb.Size.XSize
                Position = minPosition.X }
              { Size = bb.Size.YSize
                Position = minPosition.Y }
              { Size = bb.Size.ZSize
                Position = minPosition.Z } ]

        let inserter tree id =
            id |> SpatialTree.insert tree objExtentProvider

        let boundaries =
            (spaceBoundaries.Min.Get.AsArray(), spaceBoundaries.Max.Get.AsArray())
            ||> Array.zip
            |> Array.toList

        let leafCapacity = configuration.LeafCapacity
        let maxDepth = configuration.MaxDepth

        let initialTree = SpatialTree.init leafCapacity maxDepth boundaries
        let tree = ids |> Set.fold inserter initialTree

        tree
        |> SpatialTree.getObjectBuckets
        |> Seq.filter (fun bucket -> bucket.Length > 1)
        |> Seq.distinct        
        |> Seq.collect (Set.ofList >> subSetsOf2Tail >> Set.toSeq)
        |> List.ofSeq
        |> List.map SetOf2.ofSet
