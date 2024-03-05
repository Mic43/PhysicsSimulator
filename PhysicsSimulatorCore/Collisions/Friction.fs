namespace PhysicsSimulator.Collisions

open FSharpPlus
open FSharpPlus.Data
open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities

module Friction =
    open Vector3D

    let private compoundFrictionCoeff coeff1 coeff2 = max coeff1 coeff2

    let private clamped impulseValue maxFriction : State<ContactPointImpulseData, float> =
        monad {
            let! impulseAccOld = State.gets (_.AccumulatedFrictionImpulse)

            let clampedImpulseValue, newAccumulatedFrictionImpulse =
                impulseAccOld
                |> State.run (
                    impulseValue
                    |> ContactPointImpulseData.clampImpulseValue -maxFriction (maxFriction |> Some)
                )
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
        : State<ContactPointImpulseData, Vector3D> =

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

            let vRelNorm = vRel |> dotProduct normal
            let vRelTan = vRel - vRelNorm * normal
            let tangentDir = vRelTan |> normalized

            if tangentDir.Get |> isZero 0.00001  || cpImpulseData.MassTangent = 0 then
                return zero
            else
                // let K body (offset: Vector3D) =
                //     match body.MassCenter.Mass with
                //     | Mass.Infinite -> Matrix3.zero.Get
                //     | Mass.Value _ ->
                //         body.MassCenter.GetInverseMassMatrix().Get
                //         + (offset |> Matrix3.hatOperator).Get.Transpose()
                //           * body.CalcRotationalInertiaInverse().Get
                //           * (offset |> Matrix3.hatOperator).Get
                //
                // let massTangent =
                //     (K targetBody cpImpulseData.PositionOffsetFromTarget)
                //     + (K otherBody cpImpulseData.PositionOffsetFromOther)

                let massTangent = cpImpulseData.MassTangent //tangentDir.Get.Get * (massTangent * tangentDir.Get.Get)

                // printfn $"vRel: {vRel} tan dir: {tangentDir}"

                let impulseValueBase =
                    // if shouldApplyKineticFriction normal normalImpulse compoundStaticFriction then
                    //     compoundDynamicFriction * (normalImpulse |> l2Norm)
                    // else
                    (vRel  |> dotProduct tangentDir.Get) / massTangent

                let maxFriction = compoundDynamicFriction * cpImpulseData.AccumulatedNormalImpulse

                let! impulseValue = -impulseValueBase |> clamped maxFriction

                return impulseValue * tangentDir.Get
        }
