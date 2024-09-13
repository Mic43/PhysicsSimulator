namespace PhysicsSimulator.Collisions

open FSharpPlus.Lens
open Microsoft.FSharp.Collections
open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities


type BroadPhaseCollisionDetector = SimulatorObjectIdentifier Set -> SimulatorObjectIdentifier SetOf2 List

module BroadPhase =
    let dummy (ids: SimulatorObjectIdentifier Set) =
        ids |> subSetsOf2Tail |> Set.toList |> List.map SetOf2.ofSet

    let withSpatialTree
        (simulationObjectsMap: Map<SimulatorObjectIdentifier, SimulatorObject>)
        (spaceBoundaries: {| Min: Vector3D; Max: Vector3D |})
        (ids: SimulatorObjectIdentifier Set)
        =

        let objExtentProvider id =
            let bb = simulationObjectsMap[id] |> SimulatorObject.getAABoundingBox

            [ { Size = (bb |> fst).XSize
                Position = (bb |> snd).X }
              { Size = (bb |> fst).YSize
                Position = (bb |> snd).Y }
              { Size = (bb |> fst).ZSize
                Position = (bb |> snd).Z } ]

        let inserter tree id =
            id |> SpatialTree.insert tree objExtentProvider

        let boundaries =
            (spaceBoundaries.Min.Get.AsArray(), spaceBoundaries.Max.Get.AsArray())
            ||> Array.zip
            |> Array.toList

        let initialTree = SpatialTree.init 4 10 boundaries
        let tree = ids |> Set.fold inserter initialTree

        tree
        |> SpatialTree.getObjectBuckets
        |> Seq.collect (Set.ofList >> subSetsOf2Tail >> Set.toSeq)
        |> List.ofSeq
        |> List.map SetOf2.ofSet
