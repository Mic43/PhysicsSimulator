namespace PhysicsSimulator

type StepConfiguration =
    { epsilon: float          
      baumgarteTerm: float
      allowedPenetration: float
      collisionSolverIterationCount: int
      enableFriction:bool
      }

module StepConfiguration =
    let getDefault =
        { epsilon = 0.0001            
          baumgarteTerm = 0.2
          collisionSolverIterationCount = 10
          allowedPenetration = 0.001
          enableFriction = true }
