namespace PhysicsSimulator.Collisions

open PhysicsSimulator
open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities

module CollisionResponse =
    open Vector3D
    open SetOf2

    let frictionApplier = Friction.applyNoFriction
    let collisionSolverIterationCount = 10

    let private calculateRigidBodyImpulse otherBody targetBody (contactPoint: ContactPoint) =

        let compoundFriction = max targetBody.FrictionCoeff otherBody.FrictionCoeff
        let compoundElasticity = min targetBody.ElasticityCoeff otherBody.ElasticityCoeff
        let normal = contactPoint.Normal

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
            // printfn $"vNorm: {uRelNorm}"
            //
            let K body (offset: Vector3D) =
                body.MassCenter.GetInverseMassMatrix().Get
                + (offset |> Matrix3.hatOperator).Get.Transpose()
                  * body.CalcRotationalInertiaInverse().Get
                  * (offset |> Matrix3.hatOperator).Get

            let totalM = (K targetBody offset1) + (K otherBody offset2)
            let coeff = normal.Get * (totalM * normal.Get)

            let impulseValue = -(compoundElasticity + 1.0) * vRelNorm / coeff
            let impulse = impulseValue * normal

            impulse
            |> frictionApplier (totalM |> Matrix3.ofMatrix) compoundElasticity compoundFriction normal vRel vRelNorm

    let calculateImpulse (contactPoint: ContactPoint) (other: PhysicalObject) (target: PhysicalObject) =
        match (target, other) with
        | RigidBody targetBody, RigidBody otherBody -> calculateRigidBodyImpulse otherBody targetBody contactPoint

    let resolveCollision (collisionData: CollisionData) (objects: SimulatorObject SetOf2) =
        let resolveIteration (objectsPair: SimulatorObject SetOf2) =
            let resolveContactPoint objects (contactPoint: ContactPoint) =
                let offsets =
                    objects |> map (contactPoint.Position |> SimulatorObject.getOffsetFrom)

                let impulse =
                    (objects |> fst).PhysicalObject
                    |> calculateImpulse contactPoint (objects |> snd).PhysicalObject

                // printfn $"  impulse: %A{impulse} offset: {offsets}"

                ([ impulse; -impulse ] |> ofList, offsets, objects)
                |||> zip3
                |> map (fun (impulse, offset, obj) -> obj |> SimulatorObject.applyImpulse impulse offset)

            collisionData.ContactPoints |> Seq.fold resolveContactPoint objectsPair

        objects |> applyN collisionSolverIterationCount resolveIteration
