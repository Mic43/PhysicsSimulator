namespace PhysicsSimulator

open FSharpPlus
open FSharpPlus.Data
open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities
open PhysicsSimulator.Collisions

module CollisionResolver =
    open SimulatorState

    /// returns simulator state with handled collisions for colliding objects or None if there was no collision
    let private tryHandleCollision dt (collidingObjectsCandidates: SetOf2<SimulatorObject>) curSimulationState =
        let simulatorState =
            monad {
                // let! collisionData =
                match! collidingObjectsCandidates |> CollisionDetection.areColliding with
                | None -> None
                | Some collisionData ->
                    let! resolvedObjects =
                        CollisionResponse.resolveCollision
                            dt
                            collisionData
                            (collidingObjectsCandidates |> SetOf2.map (_.PhysicalObject))

                    let resolved =
                        (collidingObjectsCandidates, resolvedObjects)
                        ||> SetOf2.zip
                        |> SetOf2.map (fun p -> p ||> SimulatorObject.withPhysicalObject)
                        |> SetOf2.toList

                    let objectsIds = collidingObjectsCandidates |> SetOf2.map _.Id |> SetOf2.toSet

                    let simulatorState = curSimulationState |> changeSimulatorObjects resolved

                    { simulatorState with
                        Collisions = simulatorState.Collisions |> Map.add objectsIds collisionData }
                    |> Some
            }

        curSimulationState.Configuration |> Reader.run simulatorState

    let private withCollisionResponse dt (curSimulationState: SimulatorState) candidatesIds =

        let collidingObjectsCandidates: SetOf2<SimulatorObject> =
            candidatesIds |> SetOf2.map (updateObjectById curSimulationState dt)

        tryHandleCollision dt collidingObjectsCandidates curSimulationState
        |> Option.defaultValue curSimulationState

    let resolveAll dt collidingObjectsCandidates (curSimulationState: SimulatorState) =
        let withCollisionResponse simulatorState objectIdentifiers =
            objectIdentifiers |> SetOf2.ofSet |> withCollisionResponse dt simulatorState

        let withClearedOldCollisions =
            { curSimulationState with
                Collisions = Map.empty }

        collidingObjectsCandidates
        |> subSetsOf2Tail
        |> Set.fold withCollisionResponse withClearedOldCollisions
