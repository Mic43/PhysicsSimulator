namespace PhysicsSimulator

open System.Security.AccessControl
open MathNet.Numerics.LinearAlgebra
open Utils

module CollisionResponse =
    open Vector3D
    open SetOf2

    let private isImpulseInFrictionCone (collisionNormal: Vector3D) (frictionCoeff: float) (impulse: Vector3D) =

        let impulse = impulse.Get
        let normal = collisionNormal.Get

        let impulseNormal = normal.DotProduct(impulse) * normal
        let impulseTg = impulse - impulseNormal
        //let temp = impulse.DotProduct(normal)
        //   (impulse - temp * normal).L2Norm() <= frictionCoeff * impulse.DotProduct(normal)
        let tmp = impulse.DotProduct(normal)
        impulseTg.L2Norm() <= frictionCoeff * tmp

    let private calculateSlidingFrictionImpulse
        (massMatrix: Matrix<float>)
        elasticityCoeff
        frictionCoeff
        collisionNormal
        (uRel: Vector<float>)
        =
        let vNorm = uRel.DotProduct(collisionNormal)
        let vTan = uRel - vNorm * collisionNormal

        //TODO: vTan is zero sometimes!
        let t = vTan / vTan.L2Norm()

        let jn =
            -(elasticityCoeff + 1.0) * vNorm
            / (collisionNormal * massMatrix * (collisionNormal - frictionCoeff * t))

        jn * collisionNormal - frictionCoeff * jn * t |> Vector3D.ofVector

    let private calculateRigidBodyImpulse otherBody targetBody (contactPoint: ContactPoint) =

        let compoundFriction = max targetBody.FrictionCoeff otherBody.FrictionCoeff
        let compoundElasticity = min targetBody.ElasticityCoeff otherBody.ElasticityCoeff
        let normal = contactPoint.Normal.Get

        let offset1 =
            (contactPoint.Position, targetBody.GetMassCenterPosition) ||> apply2 (-)

        let offset2 =
            (contactPoint.Position, otherBody.GetMassCenterPosition) ||> apply2 (-)

        let vRelLinear =
            targetBody.MassCenter.Variables.Velocity.Get
            - otherBody.MassCenter.Variables.Velocity.Get

        let vRelAngular =
            (offset1 |> crossProduct (targetBody.CalcAngularVelocity()),
             (offset2 |> crossProduct (otherBody.CalcAngularVelocity())))
            ||> apply2 (-)

        let uRel = vRelLinear + vRelAngular.Get
        let uRelNorm = uRel.DotProduct(normal)
        //  printfn $"vNorm: {uRelNorm}"
        //
        let K body (offset: Vector3D) =
            body.MassCenter.GetInverseMassMatrix().Get
            + offset.HatOperator().Get.Transpose()
              * body.CalcRotationalInertiaInverse().Get
              * offset.HatOperator().Get

        let totalM = (K targetBody offset1) + (K otherBody offset2)

        let coeff = normal * (totalM * normal)

        let impulseValue = -(compoundElasticity + 1.0) * uRelNorm / coeff

        let impulse = impulseValue * normal |> ofVector
        impulse

    //    if impulse |> isImpulseInFrictionCone normal compoundFriction then
    //             impulse
    //      else
    //         (calculateSlidingFrictionImpulse totalM compoundElasticity compoundFriction normal uRel)

    let calculateImpulse (contactPoint: ContactPoint) (other: PhysicalObject) (target: PhysicalObject) =
        match (target, other) with
        | RigidBody targetBody, RigidBody otherBody -> calculateRigidBodyImpulse otherBody targetBody contactPoint

    let resolveCollision (collisionData: CollisionData) (objects: SimulatorObject SetOf2) =
        let resolveIteration (objectsPair: SimulatorObject SetOf2) =
            collisionData.ContactPoints
            |> Seq.fold
                (fun objects contactPoint ->
                    let offsets =
                        objects |> map (contactPoint.Position |> SimulatorObject.getOffsetFrom)

                    let impulse =
                        (objects |> fst).PhysicalObject
                        |> calculateImpulse contactPoint (objects |> snd).PhysicalObject

                    printfn $"  impulse: %A{impulse} offset: {offsets}"

                    let updated =
                        ([ impulse; impulse.Get.Negate() ] |> ofList, offsets, objects)
                        |||> zip3
                        |> map (fun (impulse, offset, obj) -> obj |> SimulatorObject.applyImpulse impulse offset)

                    //printfn $"  state1 after applying {updated1.PhysicalObject}"
                    //printfn $"  state2 after applying {updated2.PhysicalObject}"

                    updated)
                objectsPair

        let iterationCount = 1
        objects |> applyN iterationCount resolveIteration
