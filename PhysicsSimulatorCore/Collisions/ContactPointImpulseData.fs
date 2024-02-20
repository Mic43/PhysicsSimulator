namespace PhysicsSimulator.Collisions

open FSharpPlus
open FSharpPlus.Data
open Microsoft.VisualBasic.CompilerServices
open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities

type ContactPointImpulseData =
    { BaumgarteBias: float
      AccumulatedNormalImpulse: float
      AccumulatedFrictionImpulse: float
      PositionOffsetFromTarget: Vector3D
      PositionOffsetFromOther: Vector3D
      MassTangent: float
      MassNormal: float }

    member this.Offsets =
        (this.PositionOffsetFromTarget, this.PositionOffsetFromOther) ||> SetOf2.create

module ContactPointImpulseData =
    let init (targetBody: RigidBody) (otherBody: RigidBody) baumgarteBias (contactPoint: ContactPoint) =
        let K body (offset: Vector3D) =
            match body.MassCenter.Mass with
            | Mass.Infinite -> Matrix3.zero
            | Mass.Value _ ->
                body.MassCenter.GetInverseMassMatrix()
                + (offset |> Matrix3.hatOperator |> Matrix3.transposed)
                  * body.CalcRotationalInertiaInverse()
                  * (offset |> Matrix3.hatOperator)

        let normal = contactPoint.Normal.Get
        let offsetTarget = contactPoint.Position - targetBody.MassCenterPosition
        let offsetOther = contactPoint.Position - otherBody.MassCenterPosition

        let totalM = (K targetBody offsetTarget) + (K otherBody offsetOther)
        let massNormal = normal * (totalM * normal)

        let vRel =
            (offsetOther |> RigidBodyMotion.calculateVelocityAtOffset otherBody)
            - (offsetTarget |> RigidBodyMotion.calculateVelocityAtOffset targetBody)

        let vRelNorm = vRel |> Vector3D.dotProduct contactPoint.Normal.Get
        let vRelTan = vRel - vRelNorm * contactPoint.Normal.Get
        let tangentDir = vRelTan |> Vector3D.normalized

        let massTangent = tangentDir.Get * (totalM * tangentDir.Get)

        { BaumgarteBias = baumgarteBias
          AccumulatedNormalImpulse = 0
          PositionOffsetFromTarget = offsetTarget
          PositionOffsetFromOther = offsetOther
          MassNormal = massNormal
          AccumulatedFrictionImpulse = 0
          MassTangent = massTangent }

    let offsets (contactPointImpulseData: ContactPointImpulseData) = contactPointImpulseData.Offsets

    let clampImpulseValue minV maxV impulseValue : State<float, float> =
        let maxV = maxV |> Option.defaultValue infinity

        monad {
            let! impulseAccOld = State.get

            do! State.modify (fun oldState -> oldState + impulseValue |> clamp minV maxV)

            let! newAccumulated = State.get
            return newAccumulated - impulseAccOld
        }
