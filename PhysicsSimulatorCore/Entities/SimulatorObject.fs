namespace PhysicsSimulator.Entities

open System
open FSharpPlus.Data
open PhysicsSimulator.Utilities
open FSharpPlus

type SimulatorObjectIdentifier =
    private
    | Value of int

    member this.Get =
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
    { Id: SimulatorObjectIdentifier
      PhysicalObject: PhysicalObject
      Collider: Collider }

/// Axis aligned bounding box
type AABB = { Size: Box; CenterPosition: Vector3D }

module SimulatorObject =
    let withPhysicalObject simulatorObject physicalObject =
        { simulatorObject with
            PhysicalObject = physicalObject }

    let getOffsetFrom (point: Vector3D) (physicalObject: SimulatorObject) =
        (point, physicalObject.PhysicalObject.MassCenterPosition())
        ||> Vector3D.apply2 (-)

    let getAABoundingBox (simulatorObject: SimulatorObject) : AABB =
        match (simulatorObject.Collider, simulatorObject.PhysicalObject) with
        | Sphere sphere, Particle particle ->
            { Size = sphere.Radius |> Box.createCube
              CenterPosition = particle.Variables.Position }
        | Sphere sphere, RigidBody rigidBody ->
            { Size = sphere.Radius |> Box.createCube
              CenterPosition = rigidBody.MassCenterPosition }
        | Box box, Particle particle ->
            { Size = box
              CenterPosition = particle.Variables.Position }
        | Box box, RigidBody rigidBody ->
            { Size =
                box
                |> Box.toVector3D
                |> GraphicsUtils.toWorldCoordinates rigidBody.Variables.Orientation Vector3D.zero
                |> Box.ofVector3D
              CenterPosition = rigidBody.MassCenterPosition }

    let applyImpulse impulse offset object =
        let applyImpulseToObject object =
            match object with
            | RigidBody rigidBody -> impulse |> RigidBodyMotion.applyImpulse offset rigidBody |> RigidBody
            | Particle particle -> impulse |> ParticleMotion.applyImpulse particle |> Particle

        { object with
            PhysicalObject = object.PhysicalObject |> applyImpulseToObject }

    let update particleIntegrator rigidBodyIntegrator dt (totalForce: Vector3D) totalTorque =
        function
        | Particle p ->
            let acceleration = totalForce.Get / p.Mass.GetValue

            let particleIntegrator = particleIntegrator dt (acceleration |> Vector3D.ofVector)

            { p with
                Variables = p.Variables |> particleIntegrator }
            |> Particle
        | RigidBody rigidBody ->
            let updater =
                RigidBodyMotion.update particleIntegrator rigidBodyIntegrator totalForce totalTorque dt

            let newState = rigidBody |> updater
            newState |> RigidBody
