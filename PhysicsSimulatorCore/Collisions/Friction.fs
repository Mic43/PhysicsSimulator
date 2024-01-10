namespace PhysicsSimulator.Collisions

open PhysicsSimulator.Utilities

module Friction =
    open Vector3D

    let isImpulseInFrictionCone (collisionNormal: Vector3D) (frictionCoeff: float) (impulse: Vector3D) =

        let impulseNormal = (collisionNormal |> dotProduct impulse) * collisionNormal
        let impulseTg = impulse - impulseNormal
        //let temp = impulse.DotProduct(normal)
        //   (impulse - temp * normal).L2Norm() <= frictionCoeff * impulse.DotProduct(normal)
        let tmp = impulse |> dotProduct collisionNormal
        (impulseTg |> l2Norm) <= frictionCoeff * tmp

    let applySlidingFriction
        (massMatrix: Matrix3)
        compoundElasticity
        compoundFriction
        (collisionNormal: Vector3D)
        (vRel: Vector3D)
        (vRelNormal: float)
        impulse
        =

        let includeSlidingFriction () =
            let uTan = vRel - vRelNormal * collisionNormal
            //TODO: vTan is zero sometimes!
            let t = uTan |> normalized

            let jn =
                -(compoundElasticity + 1.0) * vRelNormal
                / (collisionNormal.Get
                   * massMatrix.Get
                   * (collisionNormal.Get - compoundFriction * t.Get))

            jn * collisionNormal - (compoundFriction * jn * t)

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


