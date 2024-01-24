namespace PhysicsSimulator

type Configuration =
    { epsilon: float          
      baumgarteTerm: float
      collisionSolverIterationCount: int
      allowedPenetration: float }

module Configuration =
    let getDefault =
        { epsilon = 0.00001            
          baumgarteTerm = 0.2
          collisionSolverIterationCount = 10
          allowedPenetration = 0.01 }
