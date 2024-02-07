namespace PhysicsSimulator

type Configuration =
    { epsilon: float          
      baumgarteTerm: float
      allowedPenetration: float
      collisionSolverIterationCount: int
      enableFriction:bool
      }

module Configuration =
    let getDefault =
        { epsilon = 0.00001            
          baumgarteTerm = 0.2
          collisionSolverIterationCount = 10
          allowedPenetration = 0.01
          enableFriction = true }
