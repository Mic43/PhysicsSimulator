namespace PhysicsSimulator.Collisions

open FSharpPlus
open FSharpPlus.Data
open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities

module Friction =
    open Vector3D

    let private compoundFrictionCoeff coeff1 coeff2 = max coeff1 coeff2

    let private clamped maxFriction (impulseValue: float SetOf2) : State<ContactPointImpulseData, float SetOf2> =
        monad {
            let! impulseAccOld = State.gets (_.AccumulatedFrictionImpulse)

            let clampedImpulseValue, newAccumulatedFrictionImpulse =
                (impulseAccOld, impulseValue)
                ||> SetOf2.zip
                |> SetOf2.map (fun (accumulated, impulse) ->
                    accumulated
                    |> State.run (
                        impulse
                        |> (ContactPointImpulseData.clampImpulseValue -maxFriction (maxFriction |> Some))
                    ))
                |> SetOf2.unzip

            do!
                State.modify (fun oldState ->
                    { oldState with
                        AccumulatedFrictionImpulse = newAccumulatedFrictionImpulse })

            return clampedImpulseValue
        }

    let private shouldApplyKineticFriction normal normalImpulse compoundStaticFriction = true

    ///normalImpulse - normal part of the collision response applied before in the same iteration of the solver
    let calculateImpulse
        (bodies: RigidBody SetOf2)
        : State<ContactPointImpulseData, Vector3D SetOf2> =

        let targetBody = bodies |> SetOf2.fst
        let otherBody = bodies |> SetOf2.snd
           
        let compoundDynamicFriction =
            targetBody.KineticFrictionCoeff
            |> compoundFrictionCoeff otherBody.KineticFrictionCoeff

        monad {
            let! cpImpulseData = State.get

            let vRel = cpImpulseData.Offsets |> RigidBodyMotion.calculateRelativeVelocity bodies
               
            let maxFriction = compoundDynamicFriction * cpImpulseData.AccumulatedNormalImpulse

            let impulsesValue =                  
                cpImpulseData.MassTangent
                |> SetOf2.zip cpImpulseData.TangentDirs
                |> SetOf2.map (fun (tanDir, massTangent) -> (vRel |> dotProduct tanDir.Get) / massTangent)

            let! clamped = impulsesValue |> (clamped maxFriction)

            return                    
                cpImpulseData.TangentDirs
                |> SetOf2.zip clamped
                |> SetOf2.map (fun (value, dir) -> value * dir.Get)
        }
