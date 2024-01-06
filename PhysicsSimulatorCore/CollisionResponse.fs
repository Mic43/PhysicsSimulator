namespace PhysicsSimulator

open System.Security.AccessControl
open MathNet.Numerics.LinearAlgebra
open Utils

module CollisionResponse =
    open Vector3D

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
            targetBody.CalcAngularVelocity().Get - otherBody.CalcAngularVelocity().Get

        let uRel = vRelLinear + vRelAngular
        let uRelNorm = uRel.DotProduct(normal)
      //  printfn $"vNorm: {uRelNorm}"
        //
        let K body (offset: Vector3D) =
            body.MassCenter.GetInverseMassMatrix().Get
            + offset.HatOperator().Get.Transpose()
              * body.CalcRotationalInertiaInverse().Get
              * offset.HatOperator().Get
        
        let totalM = (K targetBody offset1) + (K otherBody offset2)

        let coeff =  normal  * (totalM * normal)
                    
        let impulseValue =
            -(compoundElasticity + 1.0) * uRelNorm / coeff

        let impulse = impulseValue * normal |> ofVector
        impulse

    //    if impulse |> isImpulseInFrictionCone normal compoundFriction then
    //             impulse
    //      else
    //         (calculateSlidingFrictionImpulse totalM compoundElasticity compoundFriction normal uRel)

    let calculateImpulse (contactPoint: ContactPoint) (other: PhysicalObject) (target: PhysicalObject) =
        match (target, other) with
        | RigidBody targetBody, RigidBody otherBody -> calculateRigidBodyImpulse otherBody targetBody contactPoint

    let resolveCollision (collisionData: CollisionData) first second =
        //let collisionData2 = collisionData.WithInvertedNormals()

        let resolveIteration objectsPair =
            collisionData.ContactPoints
            |> Seq.fold
                (fun (first, second) contactPoint ->
                    let offset1 = contactPoint.Position |> SimulatorObject.getOffsetFrom first
                    let offset2 = contactPoint.Position |> SimulatorObject.getOffsetFrom second                  

                    let impulse =
                        first.PhysicalObject |> calculateImpulse contactPoint second.PhysicalObject
                    //  let impulse1 = impulse1.Get.Divide(collisionData.ContactPoints |> Seq.length |> float) |> ofVector


                    printfn $"  impulse: %A{impulse} offset: {offset1}"
                    //                    printfn $"  impulse second: %A{impulse2} offset: {offset2}"

                    let updated1 = first |> SimulatorObject.applyImpulse impulse offset1

                    let updated2 =
                        second |> SimulatorObject.applyImpulse (impulse.Get.Negate()) offset2

                    //printfn $"  state1 after applying {updated1.PhysicalObject}"
                    //printfn $"  state2 after applying {updated2.PhysicalObject}"

                    (updated1, updated2))
                objectsPair

        let iterationCount = 10
        (first, second) |> applyN iterationCount resolveIteration
