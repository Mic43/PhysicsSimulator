namespace PhysicsSimulator

type RigidBodyIntegratorKind =
    | FirstOrder
    | AugmentedSecondOrder

type StepConfiguration =
    { epsilon: float
      baumgarteTerm: float
      allowedPenetration: float
      collisionSolverIterationCount: int
      enableFriction: bool
      RigidBodyIntegratorKind: RigidBodyIntegratorKind }

module StepConfiguration =
    let getDefault =
        { epsilon = 0.0001
          baumgarteTerm = 0.2
          collisionSolverIterationCount = 10
          allowedPenetration = 0.001
          enableFriction = true
          RigidBodyIntegratorKind = FirstOrder }
