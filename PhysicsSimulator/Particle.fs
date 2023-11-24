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
                + acceleration * dt.TotalMilliseconds

            { Position =
                (old.Position.Get()
                 + newVelocity * dt.TotalMilliseconds)
                |> fromVector
              Velocity = newVelocity |> fromVector }
module Motion =
        
    let applyImpulse particle (impulse:Vector<float<m>>) =
        ()
                