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

module private DonkeyKongSceneData =
    type MainObjectKind =
        | Cube
        | Ball

    let mainObject = MainObjectKind.Cube

    // Zig-zag ramp like Donkey Kong: stacked levels that alternate slope
    // direction. A wall at the downhill end of every level stops the object and
    // makes it drop onto the next (opposite-sloping) level below.
    let levelCount = 4
    let segmentsPerLevel = 14
    let xHalf = 8.0
    let topZ = 17.0

    let rampAngle = 11.0 * Math.PI / 180.0
    let run = 2.0 * xHalf
    let runDrop = run * Math.Tan(rampAngle)
    let levelVerticalGap = 1.5
    let levelSpacing = runDrop + levelVerticalGap
    let totalDrop = float levelCount * levelSpacing

    let trackWidth = 0.85
    let floorThickness = 0.16
    let railHeight = 0.28
    let railThickness = 0.08
    let segmentOverlap = 0.22

    let wallThickness = 0.3
    let wallHeight = 1.6

    // Drop-off opening between the last floor and the wall so the object can
    // fall freely onto the level below instead of resting against the wall.
    let wallDropGap = 1.1

    // Several small holes per level (one segment wide each).
    let gapIndices = Set.ofList [ 2; 5; 8; 11 ]

    let cubeSize = 0.42
    let cubeMass = 50.0
    let ballRadius = 0.30
    let ballMass = 42.0

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
        staticBox xSize ySize zSize position yaw pitch roll 0.08 0.04

    let staticRail xSize ySize zSize position yaw pitch roll =
        staticBox xSize ySize zSize position yaw pitch roll 0.35 0.25

    let staticWall xSize ySize zSize position =
        staticBox xSize ySize zSize position 0.0 0.0 0.0 0.5 0.4

    let yawPitchRollFromFrame = RotationMatrix3D.yawPitchRollFromFrame

    let normalize (v: Vector3D) =
        let len = Vector3D.l2Norm v
        if len < 1e-9 then v else v / len

    /// Body X = tangent, Y = binormal, Z = normal.
    let trackFrame (tangent: Vector3D) =
        let up = Vector3D.create 0.0 0.0 1.0
        let binormal = Vector3D.crossProduct tangent up |> normalize
        let normal = Vector3D.crossProduct binormal tangent |> normalize
        binormal, normal

    let levelDirection k = if k % 2 = 0 then 1.0 else -1.0

    let levelTopZ k = topZ - float k * levelSpacing

    /// Uphill (high) start and downhill (low) end of level `k`.
    let levelEndpoints k =
        let dir = levelDirection k
        let startX = if dir > 0.0 then -xHalf else xHalf
        let endX = if dir > 0.0 then xHalf else -xHalf
        let ps = Vector3D.create startX 0.0 (levelTopZ k)
        let pe = Vector3D.create endX 0.0 (levelTopZ k - runDrop)
        ps, pe

    let createLevel k =
        let dir = levelDirection k
        let ps, pe = levelEndpoints k
        let tangent = normalize (pe - ps)
        let binormal, normal = trackFrame tangent
        let yaw, pitch, roll = yawPitchRollFromFrame tangent binormal normal

        let alongLength = Vector3D.l2Norm (pe - ps)
        let segStep = alongLength / float segmentsPerLevel
        let railOffset = trackWidth / 2.0 + railThickness / 2.0
        let railLift = floorThickness / 2.0 + railHeight / 2.0
        let lastIndex = segmentsPerLevel - 1

        // The last segment stops short of the wall (shorter length), leaving a
        // gap the object can drop through onto the level below.
        let segmentSpan i =
            let nominalStart = float i * segStep
            let nominalEnd = float (i + 1) * segStep

            let segStart = nominalStart

            let segEnd =
                if i = lastIndex then
                    max (nominalStart + 0.2) (alongLength - wallDropGap)
                else
                    nominalEnd

            let length =
                if i = lastIndex then
                    segEnd - segStart
                else
                    segStep + segmentOverlap

            let center = ps + tangent * ((segStart + segEnd) / 2.0)
            center, length

        let segments =
            [ 0 .. segmentsPerLevel - 1 ]
            |> List.filter (fun i -> not (gapIndices.Contains i))
            |> List.collect (fun i ->
                let center, segmentLength = segmentSpan i
                let floor = staticFloor segmentLength trackWidth floorThickness center yaw pitch roll

                let leftRailCenter = center + binormal * railOffset + normal * railLift
                let rightRailCenter = center - binormal * railOffset + normal * railLift

                let leftRail =
                    staticRail segmentLength railThickness railHeight leftRailCenter yaw pitch roll

                let rightRail =
                    staticRail segmentLength railThickness railHeight rightRailCenter yaw pitch roll

                [ floor; leftRail; rightRail ])

        // Wall just past the downhill edge: reverses travel onto the level below.
        let wallCenter =
            Vector3D.create
                (pe.X + dir * (wallThickness / 2.0 + 0.05))
                0.0
                (pe.Z + wallHeight / 2.0 - floorThickness / 2.0)

        let wall =
            staticWall wallThickness (trackWidth + 2.0 * railThickness) wallHeight wallCenter

        wall :: segments

    let createTrack () =
        [ 0 .. levelCount - 1 ] |> List.collect createLevel

    let cubeHalf = cubeSize / 2.0
    let startClearance = 0.03

    let topStartFrame () =
        let ps, pe = levelEndpoints 0
        let tangent = normalize (pe - ps)
        let binormal, normal = trackFrame tangent
        let yaw, pitch, roll = yawPitchRollFromFrame tangent binormal normal
        ps, normal, yaw, pitch, roll

    let topStartPosition objectHalfHeight =
        let ps, normal, _, _, _ = topStartFrame ()
        // Start a little way down the first segment so it settles on the floor.
        ps + normal * (floorThickness / 2.0 + objectHalfHeight + startClearance)

    let createCube () =
        let _, _, yaw, pitch, roll = topStartFrame ()

        { (cubeSize |> RigidBodyPrototype.createDefaultCube) with
            Mass = cubeMass |> Mass.Value
            UseGravity = true
            Position = topStartPosition cubeHalf
            Yaw = yaw
            Pitch = pitch
            Roll = roll
            StaticFrictionCoeff = 0.12
            KineticFrictionCoeff = 0.07
            ElasticityCoeff = 0.06 }

    let createBall () =
        let _, _, yaw, pitch, roll = topStartFrame ()

        { (ballRadius |> RigidBodyPrototype.createDefaultSphere) with
            Mass = ballMass |> Mass.Value
            UseGravity = true
            Position = topStartPosition ballRadius
            Yaw = yaw
            Pitch = pitch
            Roll = roll
            StaticFrictionCoeff = 0.45
            KineticFrictionCoeff = 0.35
            ElasticityCoeff = 0.10 }

    let createMainObject () =
        match mainObject with
        | Cube -> createCube ()
        | Ball -> createBall ()

    let mainObjectId = 1 |> SimulatorObjectIdentifier.fromInt

    let mainObjectMass =
        match mainObject with
        | Cube -> cubeMass
        | Ball -> ballMass

    let mainObjectHelpLabel =
        match mainObject with
        | Cube -> "szescian"
        | Ball -> "kule"

    let prototypes =
        let ground =
            { ((2.0 * xHalf + 6.0, trackWidth + 8.0, 0.5) |||> RigidBodyPrototype.createDefaultBox) with
                Mass = Mass.Infinite
                UseGravity = false
                Position = Vector3D.create 0.0 0.0 -1.0
                StaticFrictionCoeff = 0.85
                KineticFrictionCoeff = 0.70 }

        ground :: createMainObject () :: createTrack ()

    let spatialTreePadding = 2.0

    let maxFallSpeed =
        Math.Sqrt(2.0 * StepConfiguration.earthGravityAccelMagnitude * (totalDrop + 4.0))

    let impulseStrength = 145.0
    let impulseSpeedMargin = impulseStrength / mainObjectMass * 4.0

    let motionMargin =
        Vector3D.create
            (maxFallSpeed + impulseSpeedMargin)
            (trackWidth + cubeSize)
            (totalDrop + cubeSize + 3.0)

    let configuration =
        { Configuration.getDefault with
            SimulationSpeedMultiplier = 1.0
            BroadPhaseCollisionDetectionKind =
                { LeafCapacity = 4
                  SpaceBoundaries =
                    SpatialTreeBoundaries.fromPrototypeAABBs prototypes spatialTreePadding motionMargin
                  MaxDepth = 12 }
                |> BroadPhaseCollisionDetectionKind.SpatialTree
            StepConfig.GravityDirection =
                (0.0, 0.0, -1.0) |||> Vector3D.create |> NormalVector.create 0.01 }

/// Donkey Kong style zig-zag ramp: levels alternate slope direction, a wall at
/// each level end reverses travel onto the level below. Space jumps over holes.
type DonkeyKongScene() =
    inherit SceneBase(DonkeyKongSceneData.prototypes, DonkeyKongSceneData.configuration)

    static let groundObjectId = 0 |> SimulatorObjectIdentifier.fromInt
    static let mainObjectId = DonkeyKongSceneData.mainObjectId

    static let impulseDir = Vector3D.create 0.0 0.0 1.0
    static let impulseValue =
        (impulseDir |> Vector3D.normalized).Get * DonkeyKongSceneData.impulseStrength

    static let impulseOffset = Vector3D.create 0.0 0.0 0.0

    override _.GroundObjectId = groundObjectId

    override _.InitialCameraView =
        CameraView.LookAt(V3d(3.0, -28.0, 15.0), V3d(0.0, 0.0, 8.0), V3d.OOI)

    override _.HelpLines =
        [ "Scena: donkey kong (zygzak z zawracaniem)"
          $"Space - skok w gore ({DonkeyKongSceneData.mainObjectHelpLabel})" ]

    override this.OnKeyDown key =
        let simulator = this.Simulator

        match key with
        | Keys.Space ->
            transact (fun () ->
                simulator.ApplyImpulse mainObjectId impulseValue impulseOffset)
        | _ -> ()
