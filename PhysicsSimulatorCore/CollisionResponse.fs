namespace PhysicsSimulator

open MathNet.Numerics.LinearAlgebra


module CollisionResponse =
    let calculateImpulse (contactPoint: CollisionData) (target: PhysicalObject) (other: PhysicalObject)  =
        
        match (target,other) with                
            | RigidBody targetBody, RigidBody otherBody -> 
                let relativeVelocity =
                    targetBody.MassCenter.Variables.Velocity.Get()
                    - otherBody.MassCenter.Variables.Velocity.Get()

                let (normal: Vector<float>) = contactPoint.Normal |> Vector3D.toVector

                let offset =
                    targetBody.MassCenter.Variables.Position.Get() - contactPoint.ContactPoint.Get()

                let vOffset =
                    relativeVelocity + (offset
                    |> Vector3D.fromVector
                    |> Vector3D.crossProduct targetBody.Variables.AngularMomentum
                    |> Vector3D.toVector)

                let vNorm = vOffset.DotProduct(normal) * normal
                //TODO: elasticity callculation probalby wrong
                let vNormAfterCol = -targetBody.ElasticityCoeff * otherBody.ElasticityCoeff * vNorm

                let inverseRotInertia =
                    targetBody.PrincipalRotationalInertiaInverse
                    |> RigidBodyMotion.calcFullRotationalInertia targetBody.Variables.Orientation

                let offsetMatrix = (offset |> Vector3D.fromVector).HatOperator()

                let massMatrix =
                    targetBody.MassCenter.GetInverseMassMatrix().Get()
                    - offsetMatrix.Get() * inverseRotInertia.Get() * offsetMatrix.Get()
                    |> Matrix.inverse

                massMatrix * (vNormAfterCol - vNorm) |> Vector3D.fromVector
