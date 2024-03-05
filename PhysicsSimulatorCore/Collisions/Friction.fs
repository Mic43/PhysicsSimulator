namespace PhysicsSimulator.Collisions

open FSharpPlus
open FSharpPlus.Data
open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities

module Friction =
    open Vector3D

    let private compoundFrictionCoeff coeff1 coeff2 = max coeff1 coeff2

    let private clamped maxFriction (impulseValue: float SetOf2) : State<ContactPointImpulseData, float SetOf2> =
        monad {
            let! impulseAccOld = State.gets (_.AccumulatedFrictionImpulse)

            let clampedImpulseValue, newAccumulatedFrictionImpulse =
                (impulseAccOld, impulseValue)
                ||> SetOf2.zip
                |> SetOf2.map (fun (accumulated, impulse) ->
                    accumulated
                    |> State.run (
                        impulse
                        |> (ContactPointImpulseData.clampImpulseValue -maxFriction (maxFriction |> Some))
                    ))
                |> SetOf2.unzip

            do!
                State.modify (fun oldState ->
                    { oldState with
                        AccumulatedFrictionImpulse = newAccumulatedFrictionImpulse })

            return clampedImpulseValue
        }

    let private shouldApplyKineticFriction normal normalImpulse compoundStaticFriction = true

    ///normalImpulse - normal part of the collision response applied before in the same iteration of the solver
    let calculateImpulse
        (bodies: RigidBody SetOf2)
        (contactPoint: ContactPoint)
        : State<ContactPointImpulseData, Vector3D SetOf2> =

        let targetBody = bodies |> SetOf2.fst
        let otherBody = bodies |> SetOf2.snd
        let normal = contactPoint.Normal.Get

        let compoundStaticFriction =
            targetBody.StaticFrictionCoeff
            |> compoundFrictionCoeff otherBody.StaticFrictionCoeff

        let compoundDynamicFriction =
            targetBody.KineticFrictionCoeff
            |> compoundFrictionCoeff otherBody.KineticFrictionCoeff

        monad {
            let! cpImpulseData = State.get

            let vRel =
                (cpImpulseData.PositionOffsetFromOther
                 |> RigidBodyMotion.calculateVelocityAtOffset otherBody)
                - (cpImpulseData.PositionOffsetFromTarget
                   |> RigidBodyMotion.calculateVelocityAtOffset targetBody)

            let vRelNorm = (vRel |> dotProduct normal) * normal
            let vRelTan = vRel - vRelNorm
            // let tangentDir = vRelTan |> normalized
            // let tangentDir2 = vRelTan |> crossProduct vRelNorm |> normalized

            if (cpImpulseData.TangentDirs |> SetOf2.fst).Get |> isZero 0.00001 then
                return (zero, zero) |> SetOf2.ofPair
            else
                // let K body (offset: Vector3D) =
                //     match body.MassCenter.Mass with
                //     | Mass.Infinite -> Matrix3.zero.Get
                //     | Mass.Value _ ->
                //         body.MassCenter.GetInverseMassMatrix().Get
                //         + (offset |> Matrix3.hatOperator).Get.Transpose()
                //           * body.CalcRotationalInertiaInverse().Get
                //           * (offset |> Matrix3.hatOperator).Get

                // let massTangent =
                //     (K targetBody cpImpulseData.PositionOffsetFromTarget)
                //     + (K otherBody cpImpulseData.PositionOffsetFromOther)
                // let massTangent = massTangent |> Matrix3.ofMatrix
                // let massTangent = tangentDir.Get * (massTangent * tangentDir.Get)

                let massTangent = cpImpulseData.MassTangent

                // printfn $"vRel: {vRel} tan dir: {tangentDir}"

                let impulsesValueBase =
                    // if shouldApplyKineticFriction normal normalImpulse compoundStaticFriction then
                    //     compoundDynamicFriction * (normalImpulse |> l2Norm)
                    // else
                    massTangent |> SetOf2.map (fun mt -> (vRelTan |> l2Norm) / mt)

                let maxFriction = compoundDynamicFriction * cpImpulseData.AccumulatedNormalImpulse

                let! impulseValue = impulsesValueBase |> SetOf2.map (~-) |> (clamped maxFriction)

                return
                    // (tangentDir, tangentDir2)
                    // |> SetOf2.ofPair
                    cpImpulseData.TangentDirs
                    |> SetOf2.zip impulseValue
                    |> SetOf2.map (fun (value, dir) -> value * dir.Get)
        }
