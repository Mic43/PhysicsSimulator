namespace PhysicsSimulator

type PhysicalObjectIdentifier =
    private
    | Value of int

    member this.Get() =
        match this with
        | Value i -> i

    static member fromInt i =
        if i < 0 then
            invalidArg "i" "must be positive"

        i |> Value

    static member op_Implicit(i) = i |> PhysicalObjectIdentifier.fromInt

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
    let createDefaultSphere radius mass position =
        { PhysicalObject = (RigidBody.createDefaultSphere 1.0 radius mass position) |> RigidBody
          Collider = radius |> Collider.createSphere }

    let createDefaultCube size mass position =
        { PhysicalObject = (RigidBody.createDefaultBox 1.0 size size size mass position) |> RigidBody
          Collider = (size, size, size) |||> Collider.createBox }
