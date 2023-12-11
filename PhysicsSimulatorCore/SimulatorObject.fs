namespace PhysicsSimulator

type SimulatorObjectIdentifier =
    private
    | Value of int

    member this.Get() =
        match this with
        | Value i -> i

    static member fromInt i =
        if i < 0 then
            invalidArg "i" "must be positive"

        i |> Value

    static member op_Implicit(i) = i |> SimulatorObjectIdentifier.fromInt

type PhysicalObject =
    | Particle of Particle
    | RigidBody of RigidBody

    member this.AsParticle() =
        match this with
        | Particle p -> p
        | RigidBody rigidBody -> rigidBody.MassCenter

    member this.MassCenterPosition() = this.AsParticle().Variables.Position

type SimulatorObject =
    { PhysicalObject: PhysicalObject
      Collider: Collider }

module SimulatorObject =
    let defaultElasticityCoeff = 0.9
    let defaultFrictionCoeff = 0.5

    let createDefaultSphere radius mass position =
        { PhysicalObject =
            (RigidBody.createDefaultSphere defaultElasticityCoeff defaultFrictionCoeff radius mass position)
            |> RigidBody
          Collider = radius |> Collider.createSphere }

    let createDefaultCube size mass position =
        { PhysicalObject =
            (RigidBody.createDefaultBox defaultElasticityCoeff defaultFrictionCoeff size size size mass position)
            |> RigidBody
          Collider = (size, size, size) |||> Collider.createBox }
