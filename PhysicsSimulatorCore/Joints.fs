namespace PhysicsSimulator

open PhysicsSimulator.Collisions
open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities
open FSharpPlus
open FSharpPlus.Data
open Utilities.SetOf2

type BallSocketJoint =
    private
        { AnchorsOffsets: Vector3D SetOf2 }

type JointType = BallSocketJoint of BallSocketJoint

type Joint =
    { Identifiers: SimulatorObjectIdentifier SetOf2
      Type: JointType }

type JointImpulseData =
    { Bodies: RigidBody SetOf2
      AnchorsWorld: Vector3D SetOf2
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
    let createBallSocket (objects: SimulatorObject SetOf2) anchorWorldPosition =
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

    let restore dt (joint: JointType) (objects: PhysicalObject SetOf2) : Reader<StepConfiguration, SetOf2<PhysicalObject>> =
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
                    |> applyN config.collisionSolverIterationCount (restoreIteration jImpulseData joint)
                    |> map RigidBody
            }

// let body1ToBody2 = (positions |> SetOf2.snd) - (positions |> SetOf2.fst)
// let normal = body1ToBody2 |> Vector3D.normalized
// let penetration = (body1ToBody2 |> Vector3D.l2Norm) - ballSocketJoint.Distance
// let position = ((positions |> SetOf2.snd) + (positions |> SetOf2.fst)) * 0.5
//
// let collisionData =
//     { CollisionData.ContactPoints =
//         (penetration, normal, position) |||> ContactPoint.Create |> List.singleton }
//
// CollisionResponse.resolveCollision dt collisionData objects
