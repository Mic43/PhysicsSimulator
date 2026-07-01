namespace Gui.Scenes

open System

open Aardvark.Application
open Aardvark.Base
open Aardvark.Rendering
open FSharp.Data.Adaptive
open PhysicsSimulator
open PhysicsSimulator.Collisions
open PhysicsSimulator.Entities
open PhysicsSimulator.Utilities

module private SpiralSceneData =
    /// Switch here to change what rolls down the spiral (ball prototype stays in code).
    type MainObjectKind =
        | Cube
        | Ball

    let mainObject = MainObjectKind.Cube

    let segmentCount = 96
    let spiralRadius = 4.0
    let turns = 2.5
    let topZ = 10.0
    let totalDrop = 13.5

    let trackWidth = 1.1
    let floorThickness = 0.14
    let railHeight = 0.32
    let railThickness = 0.1

    let cubeSize = 0.45
    let cubeMass = 55.0
    let ballRadius = 0.32
    let ballMass = 45.0

    /// Base extension along tangent; scaled up on steeper parts of the spiral.
    let segmentOverlap = cubeSize * 1.05

    let maxDescentMultiplier = 0.9 + 1.15

    let totalAngle = turns * 2.0 * Math.PI
    let angleStep = totalAngle / float (segmentCount - 1)

    /// Steeper in the middle; higher base so the cube starts sliding from the top.
    let descentMultiplier theta =
        let t = theta / totalAngle
        0.9 + 1.15 * (Math.Sin(t * Math.PI) ** 2.0)

    let spiralPoint theta z =
        Vector3D.create (spiralRadius * Math.Cos(theta)) (spiralRadius * Math.Sin(theta)) z

    let staticBox xSize ySize zSize position yaw pitch roll staticFriction kineticFriction =
        { ((xSize, ySize, zSize) |||> RigidBodyPrototype.createDefaultBox) with
            Mass = Mass.Infinite
            UseGravity = false
            Position = position
            Yaw = yaw
            Pitch = pitch
            Roll = roll
            StaticFrictionCoeff = staticFriction
            KineticFrictionCoeff = kineticFriction
            ElasticityCoeff = 0.05 }

    let staticFloor xSize ySize zSize position yaw pitch roll =
        staticBox xSize ySize zSize position yaw pitch roll 0.12 0.08

    let staticRail xSize ySize zSize position yaw pitch roll =
        staticBox xSize ySize zSize position yaw pitch roll 0.35 0.25

    let normalize v =
        let len = Vector3D.l2Norm v

        if len < 1e-9 then
            v
        else
            v / len

    /// Body X = tangent, Y = binormal, Z = normal (matches fromYawPitchRoll).
    let yawPitchRollFromFrame = RotationMatrix3D.yawPitchRollFromFrame

    let trackFrame (theta: float) (tangent: Vector3D) =
        let up = Vector3D.create 0.0 0.0 1.0
        let radial = Vector3D.create (Math.Cos(theta)) (Math.Sin(theta)) 0.0 |> normalize
        let crossUp = Vector3D.crossProduct tangent up

        let binormal =
            if Vector3D.l2Norm crossUp < 1e-4 then
                Vector3D.crossProduct tangent radial |> normalize
            else
                crossUp |> normalize

        let normal = Vector3D.crossProduct binormal tangent |> normalize
        binormal, normal

    let dropAt theta =
        descentMultiplier theta * angleStep * (totalDrop / totalAngle)

    let spiralPointAt theta z = spiralPoint theta z

    /// Bisects the angle between incoming and outgoing helix directions (fewer steps at joints).
    let mitredTangent (pPrev: Vector3D option) (p0: Vector3D) (p1: Vector3D) (pNext: Vector3D option) =
        let chord = p1 - p0

        let tangentIn =
            match pPrev with
            | Some prev -> p0 - prev |> normalize
            | None -> chord |> normalize

        let tangentOut =
            match pNext with
            | Some next -> next - p0 |> normalize
            | None -> chord |> normalize

        tangentIn + tangentOut |> normalize

    let createTrackSegment theta z (pPrev: Vector3D option) isLast =
        let drop = dropAt theta
        let nextTheta = theta + angleStep
        let nextZ = z - drop
        let p0 = spiralPointAt theta z
        let p1 = spiralPointAt nextTheta nextZ

        let pNext =
            if isLast then
                None
            else
                let drop2 = dropAt nextTheta
                spiralPointAt (nextTheta + angleStep) (nextZ - drop2) |> Some

        let chord = p1 - p0
        let chordLength = Vector3D.l2Norm chord
        let tangent = mitredTangent pPrev p0 p1 pNext
        let steepness = descentMultiplier theta / maxDescentMultiplier
        let localOverlap = segmentOverlap * (1.0 + 0.75 * steepness)
        let segmentLength = chordLength + localOverlap
        let midTheta = theta + angleStep / 2.0
        let binormal, normal = trackFrame midTheta tangent
        let yaw, pitch, roll = yawPitchRollFromFrame tangent binormal normal

        let center = (p0 + p1) / 2.0
        let railOffset = trackWidth / 2.0 + railThickness / 2.0
        let railLift = floorThickness / 2.0 + railHeight / 2.0

        let floor = staticFloor segmentLength trackWidth floorThickness center yaw pitch roll

        let leftRailCenter =
            center + binormal * railOffset + normal * railLift

        let rightRailCenter =
            center - binormal * railOffset + normal * railLift

        let leftRail =
            staticRail segmentLength railThickness railHeight leftRailCenter yaw pitch roll

        let rightRail =
            staticRail segmentLength railThickness railHeight rightRailCenter yaw pitch roll

        floor, leftRail, rightRail, nextZ, p0

    /// Geometry of the highest track segment (theta = 0).
    let topSegmentFrame () =
        let p0 = spiralPointAt 0.0 topZ
        let p1 = spiralPointAt angleStep (topZ - dropAt 0.0)
        let p2 = spiralPointAt (2.0 * angleStep) (topZ - dropAt 0.0 - dropAt angleStep)
        let tangent = mitredTangent None p0 p1 (Some p2)
        let binormal, normal = trackFrame (angleStep / 2.0) tangent
        let yaw, pitch, roll = yawPitchRollFromFrame tangent binormal normal
        p0, normal, yaw, pitch, roll

    let createTrack () =
        let rec build i theta z pPrev acc =
            if i >= segmentCount - 1 then
                List.rev acc
            else
                let isLast = i = segmentCount - 2

                let floor, leftRail, rightRail, nextZ, p0 =
                    createTrackSegment theta z pPrev isLast

                build (i + 1) (theta + angleStep) nextZ (Some p0) (rightRail :: leftRail :: floor :: acc)

        build 0 0.0 topZ None []

    let cubeHalf = cubeSize / 2.0
    let startClearance = 0.02

    let topStartPosition objectHalfHeight =
        let p0, normal, _, _, _ = topSegmentFrame ()
        p0 + normal * (floorThickness / 2.0 + objectHalfHeight + startClearance)

    let cubeStartPosition = topStartPosition cubeHalf

    let ballStartPosition = topStartPosition ballRadius

    let createCube () =
        let _, _, yaw, pitch, roll = topSegmentFrame ()

        { (cubeSize |> RigidBodyPrototype.createDefaultCube) with
            Mass = cubeMass |> Mass.Value
            UseGravity = true
            Position = cubeStartPosition
            Yaw = yaw
            Pitch = pitch
            Roll = roll
            StaticFrictionCoeff = 0.05
            KineticFrictionCoeff = 0.01
            ElasticityCoeff = 0.08 }

    let createBall () =
        let _, _, yaw, pitch, roll = topSegmentFrame ()

        { (ballRadius |> RigidBodyPrototype.createDefaultSphere) with
            Mass = ballMass |> Mass.Value
            UseGravity = true
            Position = ballStartPosition
            Yaw = yaw
            Pitch = pitch
            Roll = roll
            StaticFrictionCoeff = 0.55
            KineticFrictionCoeff = 0.45
            ElasticityCoeff = 0.12 }

    let createMainObject () =
        match mainObject with
        | Cube -> createCube ()
        | Ball -> createBall ()

    let mainObjectId = 1 |> SimulatorObjectIdentifier.fromInt

    let mainObjectHelpLabel =
        match mainObject with
        | Cube -> "szescian"
        | Ball -> "kule"

    let prototypes =
        let ground =
            { ((18.0, 18.0, 0.5) |||> RigidBodyPrototype.createDefaultBox) with
                Mass = Mass.Infinite
                UseGravity = false
                Position = Vector3D.create 0.0 0.0 -1.0
                StaticFrictionCoeff = 0.9
                KineticFrictionCoeff = 0.75 }

        ground :: createMainObject () :: createTrack ()

    let spatialTreePadding = 2.0

    let maxFallSpeed =
        Math.Sqrt(2.0 * StepConfiguration.earthGravityAccelMagnitude * totalDrop)

    let impulseSpeedMargin = 80.0 / cubeMass * 4.0

    let motionMargin =
        Vector3D.create
            (maxFallSpeed + impulseSpeedMargin)
            (maxFallSpeed + impulseSpeedMargin)
            (totalDrop + cubeSize)

    let configuration =
        { Configuration.getDefault with
            SimulationSpeedMultiplier = 1.0
            BroadPhaseCollisionDetectionKind =
                SpatialTreeBoundaries.fromPrototypeAABBs prototypes spatialTreePadding motionMargin 4 12
            StepConfig.GravityDirection =
                (0.0, 0.0, -1.0) |||> Vector3D.create |> NormalVector.create 0.01 }

/// Spiral track from the top with side rails and a configurable main object.
type SpiralScene() =
    inherit SceneBase(SpiralSceneData.prototypes, SpiralSceneData.configuration)

    // Prototype order: ground (0), main object (1), track segments (2..).
    static let groundObjectId = 0 |> SimulatorObjectIdentifier.fromInt
    static let mainObjectId = SpiralSceneData.mainObjectId

    static let impulseStrength = 80.0
    static let impulseDir = Vector3D.create 0.0 1.0 0.0
    static let impulseValue = (impulseDir |> Vector3D.normalized).Get * impulseStrength
    static let impulseOffset = Vector3D.create 0.0 0.0 0.0

    override _.GroundObjectId = groundObjectId

    override _.InitialCameraView =
        CameraView.LookAt(V3d(10.0, -9.0, 11.0), V3d(0.0, 0.0, 6.0), V3d.OOI)

    override _.HelpLines =
        [ "Scena: spirala"
          $"Space - impuls na {SpiralSceneData.mainObjectHelpLabel}" ]

    override this.OnKeyDown key =
        let simulator = this.Simulator

        match key with
        | Keys.Space ->
            transact (fun () ->
                simulator.ApplyImpulse mainObjectId impulseValue impulseOffset)
        | _ -> ()
