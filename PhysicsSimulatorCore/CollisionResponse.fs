namespace PhysicsSimulator

open System.Security.AccessControl
open MathNet.Numerics.LinearAlgebra
open Utils

module CollisionResponse =
    open Vector3D
    open SetOf2

    let private isImpulseInFrictionCone (collisionNormal: Vector3D) (frictionCoeff: float) (impulse: Vector3D) =

        let impulseNormal = (collisionNormal |> dotProduct impulse) * collisionNormal
        let impulseTg = impulse - impulseNormal
        //let temp = impulse.DotProduct(normal)
        //   (impulse - temp * normal).L2Norm() <= frictionCoeff * impulse.DotProduct(normal)
        let tmp = impulse |> dotProduct collisionNormal
        (impulseTg |> l2Norm) <= frictionCoeff * tmp

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
        let normal = contactPoint.Normal

        let offset1 = (contactPoint.Position - targetBody.GetMassCenterPosition)
        let offset2 = (contactPoint.Position - otherBody.GetMassCenterPosition)

        let vRelLinear =
            targetBody.MassCenter.Variables.Velocity
            - otherBody.MassCenter.Variables.Velocity

        let vRelAngular =
            (offset1 |> crossProduct (targetBody.CalcAngularVelocity()))
            - (offset2 |> crossProduct (otherBody.CalcAngularVelocity()))

        let uRel = vRelLinear + vRelAngular
        let uRelNorm = uRel |> dotProduct (normal)
        //  printfn $"vNorm: {uRelNorm}"
        //
        let K body (offset: Vector3D) =
            body.MassCenter.GetInverseMassMatrix().Get
            + offset.HatOperator().Get.Transpose()
              * body.CalcRotationalInertiaInverse().Get
              * offset.HatOperator().Get

        let totalM = (K targetBody offset1) + (K otherBody offset2)

        let coeff = normal.Get * (totalM * normal.Get)

        let impulseValue = -(compoundElasticity + 1.0) * uRelNorm / coeff

        let impulse = impulseValue * normal
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
                        ([ impulse; -impulse ] |> ofList, offsets, objects)
                        |||> zip3
                        |> map (fun (impulse, offset, obj) -> obj |> SimulatorObject.applyImpulse impulse offset)

                    //printfn $"  state1 after applying {updated1.PhysicalObject}"
                    //printfn $"  state2 after applying {updated2.PhysicalObject}"

                    updated)
                objectsPair

        let iterationCount = 10
        objects |> applyN iterationCount resolveIteration
