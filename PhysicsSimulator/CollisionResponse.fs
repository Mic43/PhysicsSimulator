namespace PhysicsSimulator

open MathNet.Numerics.LinearAlgebra

type ContactPointData =
    private
        { Normal: Vector3D
          OffsetFirstBody: Vector3D
          OffsetSecondBody: Vector3D }
    static member Create normal offset1 offset2 =
        if normal |> Vector3D.toVector |> Vector.norm <> 1.0 then
            invalidArg "normal" "normal vector must me normalized"

        { ContactPointData.Normal = normal
          OffsetFirstBody = offset1
          OffsetSecondBody = offset2 }

module CollisionResponse =
    let calculateImpulse (targetBody: RigidBody) (otherBody: RigidBody) (contactPoint: ContactPointData) =
        let relativeVelocity =
            targetBody.MassCenter.ParticleVariables.Velocity.Get()
            - otherBody.MassCenter.ParticleVariables.Velocity.Get()

        let (normal: Vector<float>) =
            contactPoint.Normal |> Vector3D.toVector

        let offset =
            (contactPoint.OffsetFirstBody |> Vector3D.toVector)

        let vOffset =
            relativeVelocity + offset
            |> Vector3D.fromVector
            |> Vector3D.crossProduct targetBody.RigidBodyVariables.AngularMomentum
            |> Vector3D.toVector

        let vNorm =
            vOffset.DotProduct(normal) * normal

        let vNormAfterCol =
            -targetBody.ElasticityCoeff
            * otherBody.ElasticityCoeff
            * vNorm

        let inverseRotInertia =
            targetBody.PrincipalRotationalInertiaInverse
            |> RigidBodyMotion.calcFullRotationalInertia targetBody.RigidBodyVariables.Orientation

        let offsetMatrix =
            contactPoint.OffsetFirstBody.HatOperator()

        let massMatrix =
            targetBody.MassCenter.GetInverseMassMatrix().Get()
            - offsetMatrix.Get()
              * inverseRotInertia.Get()
              * offsetMatrix.Get()
            |> Matrix.inverse

        massMatrix * (vNormAfterCol - vNorm)
        |> Vector3D.fromVector
