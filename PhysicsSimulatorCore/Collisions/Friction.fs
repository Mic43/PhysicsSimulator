namespace PhysicsSimulator.Collisions

open PhysicsSimulator.Utilities

module Friction =
    open Vector3D

    let isImpulseInFrictionCone (collisionNormal: NormalVector) (frictionCoeff: float) (impulse: Vector3D) =

        let impulseNormal = (collisionNormal.Get |> dotProduct impulse) * collisionNormal.Get
        let impulseTg = impulse - impulseNormal
        //let temp = impulse.DotProduct(normal)
        //   (impulse - temp * normal).L2Norm() <= frictionCoeff * impulse.DotProduct(normal)
        let tmp = impulse |> dotProduct collisionNormal.Get
        (impulseTg |> l2Norm) <= frictionCoeff * tmp

    let applySlidingFriction
        (massMatrix: Matrix3)
        compoundElasticity
        compoundFriction
        (collisionNormal: NormalVector)
        (vRel: Vector3D)
        (vRelNormal: float)
        impulse
        =

        let includeSlidingFriction () =
            let uTan = vRel - vRelNormal * collisionNormal.Get            
            let t = (uTan |> normalized).Get

            let jn =
                -(compoundElasticity + 1.0) * vRelNormal
                / (collisionNormal.Get.Get
                   * massMatrix.Get
                   * (collisionNormal.Get.Get - compoundFriction * t.Get))

            jn * collisionNormal.Get// - (compoundFriction * jn * t)

        if impulse |> isImpulseInFrictionCone collisionNormal compoundFriction then
            impulse
        else
            includeSlidingFriction ()

    let applyNoFriction
        (_: Matrix3)
        _
        _
        (_: Vector3D)
        (_: Vector3D)
        (_: float)
        impulse
        =
        impulse


