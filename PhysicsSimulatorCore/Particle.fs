namespace PhysicsSimulator

open System

open FSharp.Data.UnitSystems.SI.UnitSymbols
open MathNet.Numerics.LinearAlgebra

type ParticleVariables =
    { Position: Vector3D
      Velocity: Vector3D }

type Particle =
    { ParticleVariables: ParticleVariables
      Mass: float }
    member this.GetInverseMassMatrix() =
         Matrix.Build.Diagonal(3, 3, 1.0 / this.Mass) |>  Matrix3.fromMatrix
        

type Acceleration = Vector3D
type ParticleIntegrator = TimeSpan -> Acceleration -> ParticleVariables -> ParticleVariables

module ParticleIntegrators =
    open Vector3D

    let (dummy: ParticleIntegrator) =
        fun dt acceleration vars -> vars

    let (forwardEuler: ParticleIntegrator) =
        fun dt (Vector3D acceleration) old ->
            let newVelocity =
                old.Velocity.Get()
                + acceleration * dt.TotalSeconds

            { Position =
                (old.Position.Get()
                 + newVelocity * dt.TotalSeconds)
                |> fromVector
              Velocity = newVelocity |> fromVector }

module ParticleMotion =

    let applyImpulse particle (impulse: Vector3D) : Particle =
        { particle with
            ParticleVariables =
                { particle.ParticleVariables with
                    Velocity =
                        particle.ParticleVariables.Velocity.Get()
                        + impulse.Get() / particle.Mass
                        |> Vector3D.fromVector } }
