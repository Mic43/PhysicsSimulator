namespace PhysicsSimulator.Collisions

open System
open FSharpPlus
open FSharpPlus.Data
open PhysicsSimulator
open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities

open Vector3D
open SetOf2

type ContactPointImpulseData =
    { BaumgarteBias: float
      AccumulatedNormalImpulse: float
      PositionOffsetFromTarget: Vector3D
      PositionOffsetFromOther: Vector3D
      MassNormal: float }

module ContactPointImpulseData =

    let init (targetBody: RigidBody) (otherBody: RigidBody) baumgarteBias (contactPoint: ContactPoint) =
        let K body (offset: Vector3D) =
            body.MassCenter.GetInverseMassMatrix().Get
            + (offset |> Matrix3.hatOperator).Get.Transpose()
              * body.CalcRotationalInertiaInverse().Get
              * (offset |> Matrix3.hatOperator).Get

        let normal = contactPoint.Normal.Get.Get
        let offsetTarget = contactPoint.Position - targetBody.MassCenterPosition
        let offsetOther = contactPoint.Position - otherBody.MassCenterPosition

        let totalM = (K targetBody offsetTarget) + (K otherBody offsetOther)
        let massNormal = normal * (totalM * normal)

        { BaumgarteBias = baumgarteBias
          AccumulatedNormalImpulse = 0
          PositionOffsetFromTarget = offsetTarget
          PositionOffsetFromOther = offsetOther
          MassNormal = massNormal }

type CollisionManifold =
    { Bodies: RigidBody SetOf2
      Contacts: List<ContactPoint * ContactPointImpulseData> }

module CollisionResponse =
    let frictionApplier = Friction.applyNoFriction

    let private clampImpulseValue impulseValue : State<ContactPointImpulseData, float> =
        monad {
            let! impulseAccOld = State.gets (_.AccumulatedNormalImpulse)

            do!
                State.modify (fun oldState ->
                    { oldState with
                        AccumulatedNormalImpulse = max (oldState.AccumulatedNormalImpulse + impulseValue) 0.0 })

            let! newAccumulated = State.gets (_.AccumulatedNormalImpulse)
            return newAccumulated - impulseAccOld
        }

    let private calculateBaumgarteBias config (timeInterval: TimeSpan) penetration =
        -config.baumgarteTerm / timeInterval.TotalSeconds
        * ((penetration |> abs) - config.allowedPenetration |> max 0.0)

    let private calculateRigidBodyImpulse
        targetBody
        otherBody
        (contactPoint: ContactPoint)
        : State<ContactPointImpulseData, Vector3D> =
        monad {
            let! cpImpulseData = State.get

            let compoundFriction = max targetBody.FrictionCoeff otherBody.FrictionCoeff
            let compoundElasticity = min targetBody.ElasticityCoeff otherBody.ElasticityCoeff
            let normal = contactPoint.Normal.Get

            let vRelLinear = otherBody.MassCenterVelocity - targetBody.MassCenterVelocity

            let vRelAngular =
                (RigidBody.getLinearVelocityAtOffset otherBody cpImpulseData.PositionOffsetFromOther)
                - (RigidBody.getLinearVelocityAtOffset targetBody cpImpulseData.PositionOffsetFromTarget)

            let vRel = vRelLinear + vRelAngular
            let vRelNorm = vRel |> dotProduct normal

            let bias = 0.0 //cpImpulseData.BaumgarteBias

            let impulseValue =
                (-(compoundElasticity + 1.0) * vRelNorm + bias) / cpImpulseData.MassNormal

            let! clampedImpulseValue = impulseValue |> clampImpulseValue

            clampedImpulseValue * normal
        // |> frictionApplier (totalM |> Matrix3.ofMatrix) compoundElasticity compoundFriction normal vRel vRelNorm
        }

    let resolveCollision
        dt
        (collisionData: CollisionData)
        (objects: PhysicalObject SetOf2)
        : Reader<Configuration, SetOf2<PhysicalObject>> =

        let createCollisionManifold collisionData target other config =
            { Bodies = (target, other) ||> create
              Contacts =
                collisionData.ContactPoints
                |> List.map (fun cp ->
                    let baumgarteBias = calculateBaumgarteBias config dt cp.Penetration
                    cp, cp |> ContactPointImpulseData.init target other baumgarteBias) }

        let resolveIteration (manifold: CollisionManifold) =
            let resolveContactPoint
                (contactPoint: ContactPoint)
                (objects: RigidBody SetOf2)
                : State<ContactPointImpulseData, RigidBody SetOf2> =
                monad {
                    let! offsets =
                        State.gets (fun pointImpulseData ->
                            (pointImpulseData.PositionOffsetFromTarget, pointImpulseData.PositionOffsetFromOther)
                            ||> create)

                    let! impulse = contactPoint |> calculateRigidBodyImpulse (objects |> fst) (objects |> snd)
                    impulse |> printfn "impulse value: %A"

                    return
                        ([ -impulse; impulse ] |> ofList, offsets, objects)
                        |||> zip3
                        |> map (fun (impulse, offset, rigidBody) ->
                            rigidBody |> RigidBodyMotion.applyImpulse impulse offset)
                }

            { manifold with Contacts = [] }
            |> List.foldBack
                (fun (cp, cpd) m ->
                    let updatedBodies, updatedCpImpulseData =
                        cpd |> State.run (m.Bodies |> resolveContactPoint cp)

                    { Bodies = updatedBodies
                      Contacts = (cp, updatedCpImpulseData) :: m.Contacts })
                manifold.Contacts

        let target = (objects |> fst)
        let other = (objects |> snd)

        match (target, other) with
        | RigidBody targetBody, RigidBody otherBody ->
            monad {
                let! config = ask

                let collisionManifold =
                    createCollisionManifold collisionData targetBody otherBody config

                let newManifold =
                    collisionManifold
                    |> applyN config.collisionSolverIterationCount resolveIteration

                return newManifold.Bodies |> map RigidBody
            }
