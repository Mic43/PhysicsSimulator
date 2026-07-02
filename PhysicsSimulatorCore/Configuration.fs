namespace PhysicsSimulator

open PhysicsSimulator.Utilities

type RigidBodyIntegratorKind =
    | FirstOrder
    | AugmentedSecondOrder

type SpaceBoundaries =
    { Min: Vector3D
      Max: Vector3D }

type SpatialTreeConfiguration =
    { LeafCapacity: int
      SpaceBoundaries: SpaceBoundaries
      MaxDepth: int }

    static member Default =
        { LeafCapacity = 4
          SpaceBoundaries =
            { Min = (-15.0, -15.0, -15.0) |||> Vector3D.create
              Max = (15.0, 15.0, 15.0) |||> Vector3D.create }
          MaxDepth = 10 }
type BroadPhaseCollisionDetectionKind =
    | Dummy
    | SpatialTree of SpatialTreeConfiguration 

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
