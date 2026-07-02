namespace PhysicsSimulator

open FSharpPlus
open FSharpPlus.Data
open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities

module internal JointsRestorer =
    let restoreAll dt (simState: SimulatorState) =
        let readersList =
            simState.Joints
            |> mapWith (SimulatorState.getJointObjects simState)
            |> List.map (
                Pair.map (_.PhysicalObject) |> Tuple2.mapItem2
            )
            |> List.map (fun (joint, set) -> (joint, set |> Joint.restore dt joint.Type))

        readersList
        |> List.fold
            (fun state (joint, reader) ->
                let physicalObjects = simState.Configuration |> Reader.run reader |> Pair.toList
                let simulatorObjectIdentifiers = joint.Identifiers |> Pair.toList

                state
                |> SimulatorState.changePhysicalObjects simulatorObjectIdentifiers physicalObjects)
            simState
