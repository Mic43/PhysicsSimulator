open Aardvark.Base
open Aardvark.Rendering
open FSharp.Data.Adaptive
open Aardvark.SceneGraph
open Aardvark.Application
open Aardvark.Application.Slim

open PhysicsSimulator
open PhysicsSimulator.Collisions
open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities

open Gui.Scenes
open Gui.HelpOverlay

let createScene argv : IPhysicsScene * CameraView =
    match argv with
    | [| "domino" |] ->
        DominoScene() :> IPhysicsScene,
        CameraView.LookAt(V3d(-6.0, 0.0, 2.0), V3d(0.0, 0.0, 0.8), V3d.OOI)
    | _ ->
        StackScene() :> IPhysicsScene,
        CameraView.LookAt(V3d(0.0, -2.0, 2.0), V3d.Zero, V3d.OOI)

let toTranslation (v3d: Vector3D) =
    Trafo3d.Translation(v3d.X, v3d.Y, v3d.Z)

let toRotation (simulator: Simulator) (orientationMatrix: Matrix3) =
    let matrix = orientationMatrix.Get
    let tmp = matrix.ToArray()

    let mutable m33d = tmp |> M33d.op_Explicit

    let fromM33d = Rot3d.FromM33d(m33d, simulator.Configuration.Epsilon)
    Trafo3d(fromM33d)

let getObjectTransformation (simulator: Simulator) (simObj: PhysicalObject) =
    let linearComponent = simObj.AsParticle().Variables

    let rotationalComponent =
        match simObj with
        | RigidBody rb -> rb.Variables.Orientation |> toRotation simulator
        | Particle _ -> Trafo3d(Rot3d.Identity)

    let transformation = linearComponent.Position |> toTranslation
    rotationalComponent * transformation

let objectToRenderable
    groundObjectId
    simulator
    (win: Aardvark.Glfw.Window)
    (simulatorObject: SimulatorObject)
    =
    let color =
        if simulatorObject.Id = groundObjectId then
            C4b(00, 100, 00)
        else
            C4b(100, 100, 100)

    (match simulatorObject.Collider with
     | Sphere s -> Sg.sphere' 5 color s.Radius
     | Box b ->
         let bounds = Box3d.FromCenterAndSize(V3d(0, 0, 0), V3d(b.XSize, b.YSize, b.ZSize))
         Sg.box' color bounds)
    |> Sg.transform (getObjectTransformation simulator simulatorObject.PhysicalObject)

let collisionToRenderable (win: Aardvark.Glfw.Window) collision =
    collision.ContactPoints
    |> List.map (fun cp ->
        let color = C4b(200, 0, 00)
        let pos = cp.Position
        let normal = cp.Normal.Get

        let obj =
            Sg.sphere' 5 color 0.05
            |> Sg.trafo (win.Time |> AVal.map (fun _ -> pos |> toTranslation))

        let start = V3d(pos.X, pos.Y, pos.Z)
        let endPos = start + V3d(normal.X, normal.Y, normal.Z) / 5.0

        let point2 = Sg.cone' 5 color 0.05 0.1 |> Sg.translation' endPos

        [ obj; point2 ] |> Sg.ofList)
    |> Sg.ofList

let prepareScene (win: Aardvark.Glfw.Window) (host: SceneHost) (showCollisions: aval<bool>) =
    let scene = host.Scene
    let groundObjectId = scene.GroundObjectId

    let objects =
        host.Objects
        |> AList.map (fun simulatorObject ->
            objectToRenderable groundObjectId (scene.GetSimulator()) win simulatorObject)
        |> AList.toASet
        |> Sg.set

    let collisionNodes =
        host.Collisions
        |> AList.mapA (fun collision ->
            showCollisions
            |> AVal.map (fun show ->
                if show then
                    collisionToRenderable win collision
                else
                    Sg.empty))
        |> AList.toASet
        |> Sg.set

    let collisions = collisionNodes

    seq {
        yield collisions
        yield objects
    }
    |> Sg.ofSeq

[<EntryPoint>]
let main argv =

    Aardvark.Init()

    use app = new OpenGlApplication()
    let win = app.CreateGameWindow(samples = 8)

    let scene, initialView = createScene argv
    let host = SceneHost(scene)
    host.Connect()

    let simulator = scene.GetSimulator()
    let showCollisions = cval false

    let frustum =
        win.Sizes
        |> AVal.map (fun s -> Frustum.perspective 60.0 0.1 150.0 (float s.X / float s.Y))

    let cameraView =
        DefaultCameraController.control win.Mouse win.Keyboard win.Time initialView

    let scene3d =
        prepareScene win host showCollisions
        |> Sg.effect
            [ DefaultSurfaces.trafo |> toEffect
              DefaultSurfaces.simpleLighting |> toEffect ]
        |> Sg.viewTrafo (cameraView |> AVal.map CameraView.viewTrafo)
        |> Sg.projTrafo (frustum |> AVal.map Frustum.projTrafo)

    let gui = create win

    let sg =
        [ scene3d; gui ]
        |> Sg.ofList

    let renderTask = app.Runtime.CompileRender(win.FramebufferSignature, sg)

    win.RenderTask <- renderTask

    win.Keyboard.Down.Values.Add(fun key ->
        match key with
        | Keys.C ->
            transact (fun () -> showCollisions.Value <- not showCollisions.Value)
        | Keys.R -> host.Reset()
        | Keys.Pause ->
            transact (fun () -> simulator.PauseResume ())
        | key -> scene.OnKeyDown key)

    simulator.Start()
    win.Run()
    0
