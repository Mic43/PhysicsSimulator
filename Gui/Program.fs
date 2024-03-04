// For more information see https://aka.ms/fsharp-console-apps
open System
open PhysicsSimulator
open PhysicsSimulator.Collisions
open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities

open Aardvark.Base
open Aardvark.Rendering
open FSharp.Data.Adaptive
open Aardvark.SceneGraph
open Aardvark.Application
open Aardvark.Application.Slim
open FSharpPlus

let impulsedObjectId = 1
let groundId = 0
let radius = 1.0
let mass = 100.0
let impulseStrenght = 300.0
let impulseDir = Vector3D.create 0.2 1 0.0
let impulseValue = (impulseDir |> Vector3D.normalized).Get * impulseStrenght
let impulseOffset = Vector3D.create 0.0 0.0 0
//let epsilon = 0.001

let createCube () =
    (radius |> RigidBodyPrototype.createDefaultCube)
let createSphere () =
    (radius |> RigidBodyPrototype.createDefaultSphere)
let createVerticalStack position height boxSize mass =
    [ 0 .. height - 1 ]
    |> List.map (fun i ->
        { (boxSize |> RigidBodyPrototype.createDefaultCube) with
            ElasticityCoeff = 0.1
            Mass = mass |> Mass.Value
            Position = position + Vector3D.create 0 0 (i |> float) * (boxSize + 0.01) })

let collisions: cset<CollisionData> = cset []
let objects: cset<SimulatorObject> = cset []

let prepareSimulator () =
    let rigidBodyPrototypes =
        [ { ((15.0, 15.0, 0.5) |||> RigidBodyPrototype.createDefaultBox) with
              Mass = Mass.Infinite
              UseGravity = false
              ElasticityCoeff = 0.1
              Pitch = 0.0
              Position = Vector3D.create 0 0 -0.25 }
          // { ((15.0, 15.0, 0.5) |||> RigidBodyPrototype.createDefaultBox) with
          //     Mass = Mass.Infinite
          //     UseGravity = false
          //     ElasticityCoeff = 0.1
          //     Pitch = 0.2
          //     Position = Vector3D.create -5 0 -3.5 }
          { ((0.5, 0.5, 0.5) |||> RigidBodyPrototype.createDefaultBox) with
              Mass = 50.0 |> Mass.Value
              Position = Vector3D.create 0 -3 1.25 } ]
        @ createVerticalStack ((0.0, 0.0, 1.5) |||> Vector3D.create) 0 1.0 mass

    let sim =
        Simulator(
            rigidBodyPrototypes,
            0.5,
            { Configuration.getDefault with
                baumgarteTerm = 0.2
                enableFriction = true }
        )

    sim.SimulationStateChanged.Add(fun _ ->
        transact (fun () ->
            collisions.Clear()
            collisions.AddRange sim.AllCollisions)

        transact (fun () ->
            objects.Clear()
            objects.AddRange sim.AllPhysicalObjects))

    sim

let toTranslation (v3d: Vector3D) =
    Trafo3d.Translation(v3d.X, v3d.Y, v3d.Z)
let toRotation (simulator: Simulator) (orientationMatrix: Matrix3) =
    let matrix = orientationMatrix.Get
    let tmp = matrix.ToArray()

    let mutable m33d = tmp |> M33d.op_Explicit

    let fromM33d = Rot3d.FromM33d(m33d, simulator.Configuration.epsilon)
    Trafo3d(fromM33d)
let getObjectTransformation (simulator: Simulator) (simObj:PhysicalObject) = 
    let linearComponent = simObj.AsParticle().Variables

    let rotationalComponent =
        match simObj with
        | RigidBody rb -> rb.Variables.Orientation |> toRotation simulator
        | Particle _ -> Trafo3d(Rot3d.Identity)
        
    let transformation = linearComponent.Position |> toTranslation
    rotationalComponent * transformation

let objectToRenderable simulator (win: Aardvark.Glfw.Window) simulatorObject =
    let color =
        if simulatorObject.Id = groundId then
            C4b(00, 100, 00)
        else
            C4b(100, 100, 100)

    (match simulatorObject.Collider with
     | PhysicsSimulator.Entities.Sphere s -> Sg.sphere' 5 color s.Radius
     | PhysicsSimulator.Entities.Box b ->
         // let position = simulatorObject.PhysicalObject.AsParticle().Variables.Position
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

let onKeyDown (simulator: Simulator) (key: Keys) =
    match key with
    | Keys.Space ->
        transact (fun () ->
            let physicalObjectIdentifier = impulsedObjectId |> SimulatorObjectIdentifier.fromInt
            simulator.ApplyImpulse physicalObjectIdentifier impulseValue impulseOffset)

    | Keys.Pause -> transact (fun () -> simulator.PauseResume())
    | _ -> ()

let prepareScene (win: Aardvark.Glfw.Window) sim =
    let objects = objects |> ASet.map (objectToRenderable sim win) |> Sg.set
    let collisions = collisions |> ASet.map (collisionToRenderable win) |> Sg.set

    seq {
        yield collisions
        yield objects
    }
    |> Sg.ofSeq

[<EntryPoint>]
let main _ =

    Aardvark.Init()

    use app = new OpenGlApplication()
    let win = app.CreateGameWindow(samples = 8)
    let sim = prepareSimulator ()

    let initialView = CameraView.LookAt(V3d(0.0, -2.0, 2.0), V3d.Zero, V3d.OOI)

    let frustum =
        win.Sizes
        |> AVal.map (fun s -> Frustum.perspective 60.0 0.1 150.0 (float s.X / float s.Y))

    let cameraView =
        DefaultCameraController.control win.Mouse win.Keyboard win.Time initialView

    let sg =
        prepareScene win sim
        |> Sg.effect
            [ DefaultSurfaces.trafo |> toEffect
              DefaultSurfaces.simpleLighting |> toEffect ]
        |> Sg.viewTrafo (cameraView |> AVal.map CameraView.viewTrafo)
        |> Sg.projTrafo (frustum |> AVal.map Frustum.projTrafo)

    let renderTask = app.Runtime.CompileRender(win.FramebufferSignature, sg)

    win.RenderTask <- renderTask

    (onKeyDown sim) |> win.Keyboard.Down.Values.Add

    sim.Start()
    //sim.PauseResume()
    win.Run()
    0
