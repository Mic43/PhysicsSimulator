namespace PhysicsSimulator.Collisions

open System
open FSharpPlus
open FSharpPlus.Data
open PhysicsSimulator
open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities

open Vector3D
open SetOf2

type internal CollisionManifold =
    { Bodies: RigidBody SetOf2
      TangentDirs: NormalVector SetOf2
      Contacts: List<ContactPoint * ContactPointImpulseData> }

module internal CollisionResponse =
    let private clamped impulseValue : State<ContactPointImpulseData, float> =
        monad {
            let! impulseAccOld = State.gets (_.AccumulatedNormalImpulse)

            let clampedImpulseValue, newAccumulatedNormalImpulse =
                impulseAccOld
                |> State.run (impulseValue |> ContactPointImpulseData.clampImpulseValue 0.0 None)

            do!
                State.modify (fun oldState ->
                    { oldState with
                        AccumulatedNormalImpulse = newAccumulatedNormalImpulse })

            return clampedImpulseValue
        }

    let private calculateBaumgarteBias config (timeInterval: TimeSpan) penetration =
        -config.baumgarteTerm / timeInterval.TotalSeconds
        * ((penetration + config.allowedPenetration) |> min 0.0)

    /// calculated impulse is to be applied to the first body of the bodies set
    /// providing collision normal is pointing form the first to the second body
    let private calculateNormalImpulse
        (bodies: RigidBody SetOf2)
        (contactPoint: ContactPoint)
        : State<ContactPointImpulseData, Vector3D> =

        let targetBody = bodies |> fst
        let otherBody = bodies |> snd

        monad {
            let! cpImpulseData = State.get

            if cpImpulseData.MassNormal = 0.0 then
                return zero
            else
                let compoundElasticity = min targetBody.ElasticityCoeff otherBody.ElasticityCoeff
                let normal = contactPoint.Normal.Get

                let vRel = cpImpulseData.Offsets |> RigidBodyMotion.calculateRelativeVelocity bodies                                
                let vRelNorm = vRel |> dotProduct normal
                
                let impulseValue =
                    (-(compoundElasticity + 1.0) * vRelNorm + cpImpulseData.BaumgarteBias) / cpImpulseData.MassNormal

                let! clampedImpulseValue = impulseValue |> clamped

                -clampedImpulseValue * normal
        }

    let resolveCollision
        dt
        (collisionData: CollisionData)
        (objects: PhysicalObject SetOf2)
        : Reader<StepConfiguration, SetOf2<PhysicalObject>> =

        let createCollisionManifold collisionData target other config =
            let tangentVectors =
                GraphicsUtils.computeTangentVectors collisionData.ContactPoints.Head.Normal

            { Bodies = (target, other) ||> create
              TangentDirs = tangentVectors
              Contacts =
                collisionData.ContactPoints
                |> List.map (fun cp ->
                    let baumgarteBias = calculateBaumgarteBias config dt cp.Penetration
                    cp, cp |> ContactPointImpulseData.init target other baumgarteBias tangentVectors) }

        let resolveIteration enableFriction (manifold: CollisionManifold) =
            let resolveContactPointNormal contactPoint bodies =
                monad {
                    let! offsets = State.gets ContactPointImpulseData.offsets

                    let! normalImpulse = contactPoint |> calculateNormalImpulse bodies                 

                    return
                        ([ normalImpulse; -normalImpulse ] |> ofList, offsets, bodies)
                        |||> zip3
                        |> map (fun (impulse, offset, rigidBody) ->
                            impulse |> RigidBodyMotion.applyImpulse offset rigidBody)
                }

            let resolveFriction bodies =
                monad {
                    let! offsets = State.gets ContactPointImpulseData.offsets
                    let! tangentImpulses = bodies|> Friction.calculateImpulse 
                    
                    ([ tangentImpulses; tangentImpulses |> map (~-) ] |> ofList, offsets, bodies)
                    |||> zip3
                    |> map (fun (impulses, offset, rigidBody) ->
                        rigidBody |> RigidBodyMotion.applyImpulses (impulses |> toList) offset)
                }

            let dummyFriction bodies = bodies |> State.Return

            { manifold with Contacts = [] }
            |> List.foldBack
                (fun (cp, cpd) manifold ->
                    let resolveFriction = if enableFriction then resolveFriction else dummyFriction

                    let iterationResolver =
                        manifold.Bodies |> (resolveContactPointNormal cp) |> State.bind resolveFriction

                    let updatedBodies, updatedCpImpulseData = cpd |> State.run iterationResolver

                    { manifold with
                        Bodies = updatedBodies
                        Contacts = (cp, updatedCpImpulseData) :: manifold.Contacts })
                manifold.Contacts

        match objects |> toTuple with
        | RigidBody targetBody, RigidBody otherBody ->
            monad {
                let! config = ask

                let collisionManifold =
                    createCollisionManifold collisionData targetBody otherBody config

                let newManifold =
                    collisionManifold
                    |> applyN config.collisionSolverIterationCount (resolveIteration config.enableFriction)

                return newManifold.Bodies |> map RigidBody
            }
