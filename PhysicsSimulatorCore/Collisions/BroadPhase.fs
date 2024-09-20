namespace PhysicsSimulator.Collisions

open Microsoft.FSharp.Collections
open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities


type internal BroadPhaseCollisionDetector = SimulatorObjectIdentifier Set -> SimulatorObjectIdentifier SetOf2 List

type SpatialTreeConfiguration =
    { LeafCapacity: int
      SpaceBoundaries: {| Min: Vector3D; Max: Vector3D |}
      MaxDepth: int }

    static member Default =
        { LeafCapacity = 4
          SpaceBoundaries =
            {| Min = (-15.0, -15.0, -15.0) |||> Vector3D.create
               Max = (15.0, 15.0, 15.0) |||> Vector3D.create |}
          MaxDepth = 10 }

module internal BroadPhase =
    let dummy (ids: SimulatorObjectIdentifier Set) =
        ids |> subSetsOf2Tail |> Set.toList |> List.map SetOf2.ofSet

    let withSpatialTree
        (configuration: SpatialTreeConfiguration)
        (simulationObjectsMap: Map<SimulatorObjectIdentifier, SimulatorObject>)    
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
            (configuration.SpaceBoundaries.Min.Get.AsArray(), configuration.SpaceBoundaries.Max.Get.AsArray())
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
