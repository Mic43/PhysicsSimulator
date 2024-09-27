namespace PhysicsSimulator

open PhysicsSimulator.Utilities

type RigidBodyIntegratorKind =
    | FirstOrder
    | AugmentedSecondOrder

type StepConfiguration =
    { Epsilon: float
      GravitationalAccelerationMagnitude: float
      GravityDirection: NormalVector
      BaumgarteTerm: float
      AllowedPenetration: float
      CollisionSolverIterationCount: int
      EnableFriction: bool
      RigidBodyIntegratorKind: RigidBodyIntegratorKind }

module StepConfiguration =
    let earthGravityAccelMagnitude = 9.81
    let defaultEpsilon = 0.0001
    let earthGravityDirection = (0.0, 0.0, -1.0) |||> Vector3D.create |> NormalVector.createUnsafe
    let getDefault =
        { Epsilon = defaultEpsilon
          BaumgarteTerm = 0.2
          CollisionSolverIterationCount = 10
          AllowedPenetration = 0.001
          EnableFriction = true
          RigidBodyIntegratorKind = FirstOrder
          GravitationalAccelerationMagnitude = earthGravityAccelMagnitude
          GravityDirection = earthGravityDirection }
