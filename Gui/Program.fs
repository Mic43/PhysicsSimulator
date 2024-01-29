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
let mass = 1.0
let impulseValue = Vector3D.create 0 0 -2
let impulseOffset = Vector3D.create 0 0 0
//let epsilon = 0.001

let createCube () =
    (radius |> RigidBodyPrototype.createDefaultCube)

let createSphere () =
    (radius |> RigidBodyPrototype.createDefaultSphere)

let prepareSimulator () =
    let rigidBodyPrototypes =
        [ { ((15.0, 15.0, 0.5)
             |||> Box.create
             |> RigidBodyKind.Box
             |> RigidBodyPrototype.createDefault) with
              Mass = Mass.Infinite
              UseGravity = false
              // Yaw =   -0.5
              Position = Vector3D.create 0 0 -2 }
          // { createSphere () with
          //     UseGravity = true }
          { createCube () with
              UseGravity = true
              Position = Vector3D.create 0 0 3
              ElasticityCoeff = 0.3 }
          // { createCube () with
          //     UseGravity = false
          //     ElasticityCoeff = 0.3 }

          // SimulatorObject.createDefaultCube (radius * 2.0) mass (Vector3D.create 0 0 0)
          // SimulatorObject.createDefaultCube (radius * 2.0) (mass) (Vector3D.create 3 0 0)
          //
          // SimulatorObject.createDefaultSphere radius mass Vector3D.zero
          // SimulatorObject.createDefaultSphere (radius) mass (Vector3D.create 3 0.0 0)
          // SimulatorObject.createDefaultSphere radius mass (Vector3D.create 6 0.0 0.0)
          // SimulatorObject.createDefaultSphere radius mass (Vector3D.create 9 1.0 -1.0)

          ]

    Simulator(rigidBodyPrototypes, 0.2)

let objectToRenderable (simulator: Simulator) (id: SimulatorObjectIdentifier) =
    let physicalObj = (simulator.SimulatorObject id)

    let color =
        if id = groundId then
            C4b(00, 100, 00)
        else
            C4b(100, 100, 100)

    (match physicalObj.Collider with
     | PhysicsSimulator.Entities.Sphere s -> Sg.sphere' 5 color s.Radius
     | PhysicsSimulator.Entities.Box b ->
         let position = physicalObj.PhysicalObject.AsParticle().Variables.Position
         let bounds = Box3d.FromCenterAndSize(V3d(0, 0, 0), V3d(b.XSize, b.YSize, b.ZSize))

         Sg.box' color bounds)

let collisionToRenderable (simulator: Simulator) collisionId =
    let collision = simulator.Collision collisionId
    collisionId |> printfn "%A"
    collision.ContactPoints
    |> List.map (fun cp ->
        let color = C4b(200, 0, 00)
        let pos = cp.Position
        let normal = cp.Normal.Get
        
        let obj = Sg.sphere' 5 color 0.05
        let point = Sg.translate pos.X pos.Y pos.Z obj

        let start = V3d(pos.X, pos.Y, pos.Z)
        let endPos = start + V3d(normal.X, normal.Y, normal.Z)
        //let line = Line3d(start, endPos)
        let point2 = Sg.cone' 5 color 0.05 0.1 |> Sg.translation' endPos 
      //  let a = Sg.cylinder' 5 color 0.02 1 |> Sg. 
        //let lines = Sg.lines' color [| line |]
        
        [ point;point2] |> Sg.ofList)
    |> Sg.ofList

let collisionsIds: cset<SimulatorObjectIdentifier Set> = cset []

let onKeyDown (simulator: Simulator) (key: Keys) =
    match key with
    | Keys.Space ->
        transact (fun () ->
            let physicalObjectIdentifier = impulsedObjectId |> SimulatorObjectIdentifier.fromInt
            simulator.ApplyImpulse physicalObjectIdentifier impulseValue impulseOffset)
 
    | Keys.Pause -> transact (fun () -> simulator.PauseResume())
    | _ -> ()

let getObjectTransformation (simulator: Simulator) (id: SimulatorObjectIdentifier) =
    let toTranslation (v3d: Vector3D) =
        Trafo3d.Translation(v3d.X, v3d.Y, v3d.Z)

    let toRotation (orientationMatrix: Matrix3) =
        let matrix = orientationMatrix.Get
        let tmp = matrix.ToArray()

        let mutable m33d = tmp |> M33d.op_Explicit

        let fromM33d = Rot3d.FromM33d(m33d, simulator.Configuration.epsilon)
        //let foo = fromM33d.GetEulerAngles()
        // printfn $"Angles: %A{foo.Elements.ToListOfT()}"
        //printfn $"Matrix: %A{matrix}"

        Trafo3d(fromM33d)

    let simObj = simulator.PhysicalObject id

    let linearComponent = simObj.AsParticle().Variables

    let rotationalComponent =
        match simObj with
        | RigidBody rb -> rb.Variables.Orientation |> toRotation
        | Particle _ -> Trafo3d(Rot3d.Identity)

    //linearComponent |> printfn "%A"
    transact (fun () ->
        collisionsIds.Clear()
        // simulator.CollisionsIdentifiers |> printfn "%A"
        collisionsIds.AddRange simulator.CollisionsIdentifiers)

    let transformation = linearComponent.Position |> toTranslation
    rotationalComponent * transformation

let prepareScene (win: Aardvark.Glfw.Window) sim renderablesDict =
    let getObjectTransformation = getObjectTransformation sim

    let objects =
        renderablesDict
        |> Map.map (fun id renderable ->
            renderable
            |> Sg.trafo (win.Time |> AVal.map (fun _ -> getObjectTransformation id)))

        |> Map.values

    let collisions = (collisionsIds |> ASet.map (collisionToRenderable sim) |> Sg.set)

    seq {
        yield collisions
        yield! objects
    }
    |> Sg.ofSeq

[<EntryPoint>]
let main _ =

    Aardvark.Init()

    use app = new OpenGlApplication()
    let win = app.CreateGameWindow(samples = 8)
    let sim = prepareSimulator ()

    let renderablesDict =
        sim.ObjectIdentifiers
        |> Set.toSeq
        |> Seq.map (fun key -> (key, objectToRenderable sim key))
        |> Map.ofSeq

    let initialView = CameraView.LookAt(V3d(0.0, -2.0, 2.0), V3d.Zero, V3d.OOI)

    let frustum =
        win.Sizes
        |> AVal.map (fun s -> Frustum.perspective 60.0 0.1 150.0 (float s.X / float s.Y))

    let cameraView =
        DefaultCameraController.control win.Mouse win.Keyboard win.Time initialView

    let sg =
        renderablesDict
        |> prepareScene win sim
        |> Sg.effect
            [ DefaultSurfaces.trafo |> toEffect
              DefaultSurfaces.simpleLighting |> toEffect ]
        |> Sg.viewTrafo (cameraView |> AVal.map CameraView.viewTrafo)
        |> Sg.projTrafo (frustum |> AVal.map Frustum.projTrafo)

    let renderTask = app.Runtime.CompileRender(win.FramebufferSignature, sg)

    win.RenderTask <- renderTask

    (onKeyDown sim) |> win.Keyboard.Down.Values.Add

    sim.Start()
    win.Run()
    0
