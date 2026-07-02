namespace PhysicsSimulator.Collisions

open FSharpPlus
open FSharpPlus.Data
open Microsoft.VisualBasic.CompilerServices
open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities

type internal ContactPointImpulseData =
    { BaumgarteBias: float
      AccumulatedNormalImpulse: float
      AccumulatedFrictionImpulse: float Pair
      PositionOffsetFromTarget: Vector3D
      PositionOffsetFromOther: Vector3D
      MassTangent: float Pair
      TangentDirs: NormalVector Pair
      MassNormal: float }

    member this.Offsets =
        (this.PositionOffsetFromTarget, this.PositionOffsetFromOther) ||> Pair.create

module internal ContactPointImpulseData =
    let init
        (targetBody: RigidBody)
        (otherBody: RigidBody)
        baumgarteBias
        (tangentVectors: NormalVector Pair)
        (contactPoint: ContactPoint)
        =
     
        let normal = contactPoint.Normal.Get
        let offsetTarget = contactPoint.Position - targetBody.MassCenterPosition
        let offsetOther = contactPoint.Position - otherBody.MassCenterPosition

        let totalM =
            (offsetTarget, offsetOther)
            |> Pair.ofPair
            |> RigidBodyMotion.calculateTranslationConstraintMass ((targetBody, otherBody) |> Pair.ofPair)

        let massNormal = normal * (totalM * normal)

        let massTangent =
            tangentVectors |> Pair.map (_.Get >> (fun dir -> dir * (totalM * dir)))

        { BaumgarteBias = baumgarteBias
          AccumulatedNormalImpulse = 0
          PositionOffsetFromTarget = offsetTarget
          PositionOffsetFromOther = offsetOther
          MassNormal = massNormal
          AccumulatedFrictionImpulse = (0.0, 0.0) |> Pair.ofPair
          MassTangent = massTangent
          TangentDirs = tangentVectors }

    let offsets (contactPointImpulseData: ContactPointImpulseData) = contactPointImpulseData.Offsets

    let clampImpulseValue minV maxV impulseValue : State<float, float> =
        let maxV = maxV |> Option.defaultValue infinity

        monad {
            let! impulseAccOld = State.get

            do! State.modify (fun oldAccumulated -> oldAccumulated + impulseValue |> clamp minV maxV)

            let! newAccumulated = State.get
            return newAccumulated - impulseAccOld
        }
