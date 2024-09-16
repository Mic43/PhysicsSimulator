namespace PhysicsSimulator

open FSharpPlus
open FSharpPlus.Data
open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities
open PhysicsSimulator.Collisions

module internal CollisionResolver =
    open SimulatorState

    /// returns simulator state with handled collisions for colliding objects or None if there was no collision
    let private tryHandleCollision dt (collidingObjectsCandidates: SetOf2<SimulatorObject>) curSimulationState =
        let nextSimulationState =
            monad {
                match! collidingObjectsCandidates |> CollisionDetection.areColliding with
                | None -> None
                | Some collisionData ->
                    let! resolvedObjects =
                        CollisionResponse.resolveCollision
                            dt
                            collisionData
                            (collidingObjectsCandidates |> SetOf2.map (_.PhysicalObject))

                    let objectsIds = collidingObjectsCandidates |> SetOf2.map _.Id |> SetOf2.toSet

                    let simulatorState =
                        curSimulationState
                        |> changePhysicalObjects
                            (collidingObjectsCandidates |> SetOf2.toList |> List.map (_.Id))
                            (resolvedObjects |> SetOf2.toList)

                    { simulatorState with
                        Collisions = simulatorState.Collisions |> Map.add objectsIds collisionData }
                    |> Some
            }

        curSimulationState.Configuration |> Reader.run nextSimulationState

    let private withCollisionResponse dt (curSimulationState: SimulatorState) candidatesIds =

        let collidingObjectsCandidates: SetOf2<SimulatorObject> =
            candidatesIds |> SetOf2.map (updateObjectById curSimulationState dt)

        tryHandleCollision dt collidingObjectsCandidates curSimulationState
        |> Option.defaultValue curSimulationState

    let resolveAll (broadPhaseCollisionDetection: BroadPhaseCollisionDetector) dt (curSimulationState: SimulatorState) =        
        let withClearedOldCollisions =
            { curSimulationState with
                Collisions = Map.empty }

        curSimulationState.Objects
        |> Map.keys
        |> Set.ofSeq
        |> broadPhaseCollisionDetection
        |> List.fold (withCollisionResponse dt) withClearedOldCollisions
