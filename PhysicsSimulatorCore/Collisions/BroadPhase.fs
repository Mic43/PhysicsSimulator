namespace PhysicsSimulator.Collisions

open Microsoft.FSharp.Collections
open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities


type BroadPhaseCollisionDetector = SimulatorObjectIdentifier Set -> SimulatorObjectIdentifier SetOf2 List


module BroadPhase =
    let dummy (ids: SimulatorObjectIdentifier Set) =
        ids |> subSetsOf2Tail |> Set.toList |> List.map SetOf2.ofSet

    let withSpatialTree
        (simulationObjectsMap: Map<SimulatorObjectIdentifier, SimulatorObject>)
        (ids: SimulatorObjectIdentifier Set)
        =
        dummy ids
