namespace PhysicsSimulator.Collisions

open FSharpPlus
open FSharpPlus.Data
open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities

module internal Friction =
    open Vector3D

    let private compoundFrictionCoeff coeff1 coeff2 = max coeff1 coeff2

    let private clamped maxFriction (impulseValue: float Pair) : State<ContactPointImpulseData, float Pair> =
        monad {
            let! impulseAccOld = State.gets (_.AccumulatedFrictionImpulse)

            let clampedImpulseValue, newAccumulatedFrictionImpulse =
                (impulseAccOld, impulseValue)
                ||> Pair.zip
                |> Pair.map (fun (accumulated, impulse) ->
                    accumulated
                    |> State.run (
                        impulse
                        |> (ContactPointImpulseData.clampImpulseValue -maxFriction (maxFriction |> Some))
                    ))
                |> Pair.unzip

            do!
                State.modify (fun oldState ->
                    { oldState with
                        AccumulatedFrictionImpulse = newAccumulatedFrictionImpulse })

            return clampedImpulseValue
        }

    let private shouldApplyKineticFriction normal normalImpulse compoundStaticFriction = true

    ///normalImpulse - normal part of the collision response applied before in the same iteration of the solver
    let calculateImpulse
        (bodies: RigidBody Pair)
        : State<ContactPointImpulseData, Vector3D Pair> =

        let targetBody = bodies |> Pair.fst
        let otherBody = bodies |> Pair.snd
           
        let compoundDynamicFriction =
            targetBody.KineticFrictionCoeff
            |> compoundFrictionCoeff otherBody.KineticFrictionCoeff

        monad {
            let! cpImpulseData = State.get

            let vRel = cpImpulseData.Offsets |> RigidBodyMotion.calculateRelativeVelocity bodies
               
            let maxFriction = compoundDynamicFriction * cpImpulseData.AccumulatedNormalImpulse

            let impulsesValue =                  
                cpImpulseData.MassTangent
                |> Pair.zip cpImpulseData.TangentDirs
                |> Pair.map (fun (tanDir, massTangent) -> (vRel |> dotProduct tanDir.Get) / massTangent)

            let! clamped = impulsesValue |> (clamped maxFriction)

            return                    
                cpImpulseData.TangentDirs
                |> Pair.zip clamped
                |> Pair.map (fun (value, dir) -> value * dir.Get)
        }
