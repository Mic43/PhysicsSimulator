namespace PhysicsSimulator

open MathNet.Numerics.LinearAlgebra


type RigidBody =
    private
        { Orientation: Matrix3
          MassCenter: Particle
          RotationalInertia: Matrix3
          AngularMomentum: Vector3D }

module RigidBody =
    let createBox initialOrientation initialVelocity sizeX sizeY sizeZ mass position =
        let m = mass / 12.0
        let a2 = sizeX * sizeX
        let b2 = sizeY * sizeY
        let c2 = sizeZ * sizeY

        { Orientation = initialOrientation
          RigidBody.MassCenter =
            { ParticleVariables =
                { ParticleVariables.Position = position
                  Velocity = initialVelocity }
              Particle.Mass = mass }
          RotationalInertia =
            Matrix<float>.Build.Diagonal
                [| m * (b2 + c2)
                   m * (a2 + c2)
                   m * (a2 + b2) |]
            |> Matrix3.Value
          AngularMomentum = Vector3D.zero }

    let createDefaultBox =
        createBox RotationMatrix3D.zero Vector3D.zero

    let createSphere initialOrientation initialVelocity radius mass position =
        let I = 2.0 * mass * radius * radius / 5.0

        { Orientation = initialOrientation
          RigidBody.MassCenter =
            { ParticleVariables =
                { ParticleVariables.Position = position
                  Velocity = initialVelocity }
              Particle.Mass = mass }
          RotationalInertia =
            Matrix<float>.Build.Diagonal (3, 3, I)
            |> Matrix3.Value
          AngularMomentum = Vector3D.zero }
    let createDefaultSphere = createSphere RotationMatrix3D.zero Vector3D.zero