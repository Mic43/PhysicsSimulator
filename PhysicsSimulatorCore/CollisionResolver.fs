namespace PhysicsSimulator

open FSharpPlus
open FSharpPlus.Data
open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities
open PhysicsSimulator.Collisions

module internal CollisionResolver =
    open SimulatorState

    /// returns simulator state with handled collisions for colliding objects or None if there was no collision
    let private tryHandleCollision dt (collidingObjectsCandidates: Pair<SimulatorObject>) curSimulationState =
        let nextSimulationState =
            monad {
                match! collidingObjectsCandidates |> CollisionDetection.areColliding with
                | None -> None
                | Some collisionData ->
                    let! resolvedObjects =
                        CollisionResponse.resolveCollision
                            dt
                            collisionData
                            (collidingObjectsCandidates |> Pair.map (_.PhysicalObject))

                    let objectsIds = collidingObjectsCandidates |> Pair.map _.Id |> Pair.toSet

                    let simulatorState =
                        curSimulationState
                        |> changePhysicalObjects
                            (collidingObjectsCandidates |> Pair.toList |> List.map (_.Id))
                            (resolvedObjects |> Pair.toList)

                    { simulatorState with
                        Collisions = simulatorState.Collisions |> Map.add objectsIds collisionData }
                    |> Some
            }

        curSimulationState.Configuration |> Reader.run nextSimulationState

    let private withCollisionResponse dt (curSimulationState: SimulatorState) candidatesIds =

        let collidingObjectsCandidates: Pair<SimulatorObject> =
            candidatesIds |> Pair.map (updateObjectById curSimulationState dt)

        tryHandleCollision dt collidingObjectsCandidates curSimulationState
        |> Option.defaultValue curSimulationState


    let resolveAll dt (curSimulationState: SimulatorState) =
        let withClearedOldCollisions =
            { curSimulationState with
                Collisions = Map.empty }

        let withUpdatedBroadCollisions data ss =
            { ss with
                BroadPhaseCollisionDetectorData = data }

        let canIntersect (candidate: Pair<SimulatorObjectIdentifier>) =
            candidate.Get
            |> List.exists (fun id -> curSimulationState.Objects[id].PhysicalObject.IsStatic() |> not)

        let collisionsCandidates, broadPhaseCollisionDetectorData =
            curSimulationState.Objects |> BroadPhase.update |> State.run
            <| curSimulationState.BroadPhaseCollisionDetectorData

        collisionsCandidates
        |> List.filter canIntersect
        |> List.fold (withCollisionResponse dt) withClearedOldCollisions
        |> withUpdatedBroadCollisions broadPhaseCollisionDetectorData
