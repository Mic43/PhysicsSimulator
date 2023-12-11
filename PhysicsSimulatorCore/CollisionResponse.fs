namespace PhysicsSimulator

open MathNet.Numerics.LinearAlgebra


module CollisionResponse =

    let private isImpulseInFrictionCone (collisionNormal: Vector3D) (frictionCoeff: float) (impulse: Vector3D) =

        let impulse = impulse.Get()
        let normal = collisionNormal.Get()

        (impulse - impulse.DotProduct(normal) * normal).L2Norm()
        <= frictionCoeff * impulse.DotProduct(normal)

    let private calculateSlidingFrictionImpulse
        massMatrix
        elasticityCoeff
        frictionCoeff
        collisionNormal
        (vOffset: Vector<float>)
        =
        let vNorm = vOffset.DotProduct(collisionNormal)
        let vTan = vOffset - vNorm * collisionNormal
        let t = vTan / vTan.L2Norm()

        let jn =
            -(elasticityCoeff + 1.0) * vNorm
            / (collisionNormal * massMatrix * (collisionNormal - frictionCoeff * t))

        jn * collisionNormal - frictionCoeff * jn * t |> Vector3D.ofVector

    let calculateImpulse (collisionData: CollisionData) (target: PhysicalObject) (other: PhysicalObject) =

        match (target, other) with
        | RigidBody targetBody, RigidBody otherBody ->
            let relativeVelocity =
                targetBody.MassCenter.Variables.Velocity.Get()
                - otherBody.MassCenter.Variables.Velocity.Get()

            let compoundFriction = max targetBody.FrictionCoeff otherBody.FrictionCoeff
            let compoundElasticity = min targetBody.ElasticityCoeff otherBody.ElasticityCoeff

            let normal = collisionData.Normal.Get()

            let offset =
                targetBody.MassCenter.Variables.Position.Get() - collisionData.ContactPoint.Get()

            let vOffset = // velocity of colliding point before collision
                relativeVelocity
                + (offset |> Vector3D.crossProductV (targetBody.Variables.AngularMomentum.Get()))

            let vNorm = vOffset.DotProduct(normal) * normal // normal component of velocity before collision

            let vNormAfterCol = -compoundElasticity * vNorm

            let inverseRotInertia =
                targetBody.PrincipalRotationalInertiaInverse
                |> RigidBodyMotion.calcFullRotationalInertia targetBody.Variables.Orientation

            let offsetMatrix = (offset |> Vector3D.ofVector).HatOperator()

            let massMatrix =
                targetBody.MassCenter.GetInverseMassMatrix().Get()
                - offsetMatrix.Get() * inverseRotInertia.Get() * offsetMatrix.Get()
                |> Matrix.inverse

            let impulse = massMatrix * (vNormAfterCol - vNorm) |> Vector3D.ofVector

            if impulse |> isImpulseInFrictionCone normal compoundFriction then
                impulse
            else       
                 (calculateSlidingFrictionImpulse massMatrix compoundElasticity compoundFriction normal vOffset)
