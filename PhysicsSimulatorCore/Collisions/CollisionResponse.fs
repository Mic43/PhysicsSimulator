namespace PhysicsSimulator.Collisions

open System
open FSharpPlus
open FSharpPlus.Data
open PhysicsSimulator
open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities

module CollisionResponse =
    open Vector3D
    open SetOf2

    let frictionApplier = Friction.applyNoFriction
    // let collisionSolverIterationCount = Configuration.collisionSolverIterationCount

    let private calculateBaumgarteBias (timeInterval: TimeSpan) penetration : Reader<Configuration, float> =
        monad {
            let! config = ask

            return
                -config.baumgarteTerm / timeInterval.TotalSeconds
                * ((penetration |> abs) - config.allowedPenetration |> max 0.0)
        }

    let private calculateRigidBodyImpulse
        timeInterval
        (otherBody: RigidBody)
        (targetBody: RigidBody)
        (contactPoint: ContactPoint)
        =

        let compoundFriction = max targetBody.FrictionCoeff otherBody.FrictionCoeff
        let compoundElasticity = min targetBody.ElasticityCoeff otherBody.ElasticityCoeff
        let normal = contactPoint.Normal.Get

        let offset1 = (contactPoint.Position - targetBody.GetMassCenterPosition)
        let offset2 = (contactPoint.Position - otherBody.GetMassCenterPosition)

        let vRelLinear =
            targetBody.MassCenter.Variables.Velocity
            - otherBody.MassCenter.Variables.Velocity

        let vRelAngular =
            (offset1 |> crossProduct (targetBody.CalcAngularVelocity()))
            - (offset2 |> crossProduct (otherBody.CalcAngularVelocity()))

        let vRel = vRelLinear + vRelAngular
        let vRelNorm = vRel |> dotProduct normal

        if vRelNorm < 0 then
            zero
        else
            let K body (offset: Vector3D) =
                body.MassCenter.GetInverseMassMatrix().Get
                + (offset |> Matrix3.hatOperator).Get.Transpose()
                  * body.CalcRotationalInertiaInverse().Get
                  * (offset |> Matrix3.hatOperator).Get

            //TODO: should be calculated only once, not for every iteration
            let totalM = (K targetBody offset1) + (K otherBody offset2)
            let coeff = normal.Get * (totalM * normal.Get)
            //TODO: should be calculated only once, not for every iteration
            let bias = 0.0 // calculateBaumgarteBias timeInterval contactPoint.Penetration
            printf $"Bias {bias}"

            let impulseValue = (-(compoundElasticity + 1.0) * vRelNorm + bias) / coeff
            let impulse = impulseValue * normal

            impulse
            |> frictionApplier (totalM |> Matrix3.ofMatrix) compoundElasticity compoundFriction normal vRel vRelNorm

    let private calculateImpulse dt (contactPoint: ContactPoint) (other: PhysicalObject) (target: PhysicalObject) =
        match (target, other) with
        | RigidBody targetBody, RigidBody otherBody -> calculateRigidBodyImpulse dt otherBody targetBody contactPoint

    let resolveCollision
        dt
        (collisionData: CollisionData)
        (objects: SimulatorObject SetOf2)
        : Reader<Configuration, SetOf2<SimulatorObject>> =

        let resolveIteration (objectsPair: SimulatorObject SetOf2) =
            let resolveContactPoint objects (contactPoint: ContactPoint) =
                let offsets =
                    objects |> map (contactPoint.Position |> SimulatorObject.getOffsetFrom)

                let impulse =
                    (objects |> fst).PhysicalObject
                    |> calculateImpulse dt contactPoint (objects |> snd).PhysicalObject
             
                ([ impulse; -impulse ] |> ofList, offsets, objects)
                |||> zip3
                |> map (fun (impulse, offset, obj) -> obj |> SimulatorObject.applyImpulse impulse offset)

            collisionData.ContactPoints |> Seq.fold resolveContactPoint objectsPair

        monad {
            let! config = ask
            return objects |> applyN config.collisionSolverIterationCount resolveIteration
        }
