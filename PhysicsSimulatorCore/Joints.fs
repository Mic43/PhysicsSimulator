namespace PhysicsSimulator

open PhysicsSimulator.Collisions
open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities
open FSharpPlus
open FSharpPlus.Data
open Utilities.Pair

type BallSocketJoint =
    private
        { AnchorsOffsets: Vector3D Pair }

type JointType = BallSocketJoint of BallSocketJoint

type Joint =
    { Identifiers: SimulatorObjectIdentifier Pair
      Type: JointType }

type JointImpulseData =
    { Bodies: RigidBody Pair
      AnchorsWorld: Vector3D Pair
      ConstraintMass: Matrix3 }

module JointImpulseData =
    let init bodies jointType =
        let anchors =
            match jointType with
            | BallSocketJoint bsj ->
                ((bodies, bsj.AnchorsOffsets)
                 ||> zip
                 |> map (fun (body, anchor) -> body.Variables.Orientation * anchor))

        let constraintMass =
            match jointType with
            | BallSocketJoint bsj -> RigidBodyMotion.calculateTranslationConstraintMass bodies anchors

        { Bodies = bodies
          ConstraintMass = constraintMass
          AnchorsWorld = anchors }

module Joint =
    let createBallSocket (objects: SimulatorObject Pair) anchorWorldPosition =
        match (objects |> map (_.PhysicalObject) |> toTuple) with
        | RigidBody body1, RigidBody body2 ->
            { Identifiers = objects |> map (_.Id)
              Type =
                { AnchorsOffsets =
                    [ body1; body2 ]
                    |> mapWith (fun body -> anchorWorldPosition - body.MassCenterPosition)
                    |> List.map (fun (body, pos) -> (body.Variables.Orientation |> Matrix3.transposed) * pos)
                    |> ofList }
                |> BallSocketJoint }

    let private calculateRestoringImpulse jointImpulseData =
        function
        | BallSocketJoint bsj ->
            let vRel =
                bsj.AnchorsOffsets
                |> RigidBodyMotion.calculateRelativeVelocity jointImpulseData.Bodies

            let bias = Vector3D.zero
            jointImpulseData.ConstraintMass * (bias - vRel)

    let restore dt (joint: JointType) (objects: PhysicalObject Pair) : Reader<StepConfiguration, Pair<PhysicalObject>> =
        match (objects |> toTuple) with
        | RigidBody body1, RigidBody body2 ->
            let restoreIteration jointImpulseData jointType bodies =
                match jointType with
                | BallSocketJoint bsj ->
                    let impulse = joint |> calculateRestoringImpulse jointImpulseData

                    ((impulse, -impulse) |> ofPair, bsj.AnchorsOffsets, bodies)
                    |||> zip3
                    |> map (fun (impulse, offset, rigidBody) ->
                        impulse |> RigidBodyMotion.applyImpulse offset rigidBody)

            // match joint with
            // | BallSocketJoint ballSocketJoint ->
            // let positions =
            //     (ballSocketJoint.AnchorsOffsets, (body1, body2) |> ofPair)
            //     ||> zip
            //     |> map (fun (offset, body) -> offset |> RigidBody.toWorldCoordinates body)
            let bodies = (body1, body2) |> ofPair
            let jImpulseData = joint |> JointImpulseData.init bodies

            monad {
                let! config = ask

                return
                    bodies
                    |> applyN config.CollisionSolverIterationCount (restoreIteration jImpulseData joint)
                    |> map RigidBody
            }

// let body1ToBody2 = (positions |> Pair.snd) - (positions |> Pair.fst)
// let normal = body1ToBody2 |> Vector3D.normalized
// let penetration = (body1ToBody2 |> Vector3D.l2Norm) - ballSocketJoint.Distance
// let position = ((positions |> Pair.snd) + (positions |> Pair.fst)) * 0.5
//
// let collisionData =
//     { CollisionData.ContactPoints =
//         (penetration, normal, position) |||> ContactPoint.Create |> List.singleton }
//
// CollisionResponse.resolveCollision dt collisionData objects
