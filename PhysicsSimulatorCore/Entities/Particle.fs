namespace PhysicsSimulator.Entities

open System

open MathNet.Numerics.LinearAlgebra
open PhysicsSimulator.Utilities

type ParticleVariables =
    { Position: Vector3D
      Velocity: Vector3D }

type Particle =
    { Variables: ParticleVariables
      Mass: Mass }

    member this.GetInverseMassMatrix() =
        Matrix.Build.Diagonal(3, 3, 1.0 / this.Mass.GetValue) |> Matrix3.ofMatrix

type ParticleIntegrator = TimeSpan -> Acceleration -> ParticleVariables -> ParticleVariables

module ParticleIntegrators =

    let (dummy: ParticleIntegrator) = fun dt acceleration vars -> vars

    let (forwardEuler: ParticleIntegrator) =
        fun dt acceleration old ->
            let newVelocity = old.Velocity + acceleration * dt.TotalSeconds

            { Position = old.Position + newVelocity * dt.TotalSeconds
              Velocity = newVelocity }

module ParticleMotion =

    let applyImpulse particle (impulse: Vector3D) : Particle =
        { particle with
            Variables.Velocity = particle.Variables.Velocity + impulse / particle.Mass.GetValue }
