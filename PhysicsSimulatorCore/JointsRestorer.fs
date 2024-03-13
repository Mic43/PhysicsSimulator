namespace PhysicsSimulator

open FSharpPlus
open FSharpPlus.Data
open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities

module JointsRestorer =
    let restoreAll dt (simState: SimulatorState) =
        let readersList =
            simState.Joints
            |> mapWith (SimulatorState.getJointObjects simState)
            |> List.map (
                SetOf2.map (_.PhysicalObject) |> Tuple2.mapItem2
            )
            |> List.map (fun (joint, set) -> (joint, set |> Joint.restore dt joint.Type))

        readersList
        |> List.fold
            (fun state (joint, reader) ->
                let physicalObjects = simState.Configuration |> Reader.run reader |> SetOf2.toList
                let simulatorObjectIdentifiers = joint.Identifiers |> SetOf2.toList

                state
                |> SimulatorState.changePhysicalObjects simulatorObjectIdentifiers physicalObjects)
            simState
