// For more information see https://aka.ms/fsharp-console-apps
open System
open Aardvark.Base
open Aardvark.Glfw
open Aardvark.Rendering
open FSharp.Data.Adaptive
open Aardvark.SceneGraph
open Aardvark.Application
open Aardvark.Application.Slim
open PhysicsSimulator
open FSharpPlus

let radius = 0.5

let prepareSimulator () =

    [ SimulatorObject.createDefaultSphere radius Vector3D.zero
      SimulatorObject.createDefaultSphere radius (Vector3D.create 1.0 1.0 1.0) ]
    |> Simulator

let getObjectTransformation (simulator: SimulatorData ref) (id: PhysicalObjectIdentifier) =
    // let startTime = System.DateTime.Now

    let toTranslation (v3d: Vector3D) =
        Trafo3d.Translation(v3d.X, v3d.Y, v3d.Z)
    // let toRotation (orientationMatrix:Matrix3) =
    //     let matrix = orientationMatrix.Get().
    //     Trafo3d(Rot3d.FromM33d

    let simObj =
        simulator.Value.GetObjects()[id]

    let pos =
        (match simObj.PhysicalObject with
         | RigidBody rb -> rb.MassCenter.ParticleVariables
         | Particle p -> p.ParticleVariables)
            .Position

    let transformation = pos |> toTranslation
    transformation

let toRenderable (simulator: SimulatorData ref) (id: PhysicalObjectIdentifier) =
    let physicalObj =
        (simulator.Value.GetObjects()[id]).PhysicalObject

    let transformation =
        getObjectTransformation simulator id

    (match physicalObj with
     | RigidBody rb ->
         let color = C4b(100, 100, 100)
         //Sg.box' color bounds
         Sg.sphere' 5 color radius)
    |> Sg.transform transformation


[<EntryPoint>]
let main _ =

    Aardvark.Init()

    use app = new OpenGlApplication()
    let win = app.CreateGameWindow(samples = 8)

    let initialView =
        CameraView.LookAt(V3d(2.0, 2.0, 2.0), V3d.Zero, V3d.OOI)

    let frustum =
        win.Sizes
        |> AVal.map (fun s -> Frustum.perspective 60.0 0.1 50.0 (float s.X / float s.Y))

    let cameraView =
        DefaultCameraController.control win.Mouse win.Keyboard win.Time initialView

    let sim = prepareSimulator ()
    let simData = sim.SimulatorData

    let renderablesDict =
        simData.Value.GetObjects()
        |> Map.map (fun key _ -> toRenderable simData key)

    sim.StartUpdateThread()
    
    let getObjectTransformation =
        getObjectTransformation simData 

    let sg =
        renderablesDict
        |> Map.map (fun id renderable ->
            renderable
            |> Sg.trafo (
                win.Time
                |> AVal.map (fun _ -> getObjectTransformation id)
            ))
        |> Map.values
        |> Sg.ofSeq
        |> Sg.effect [ DefaultSurfaces.trafo |> toEffect
                       DefaultSurfaces.constantColor C4f.Red |> toEffect
                       //  DefaultSurfaces.vertexColor |> toEffect
                       DefaultSurfaces.simpleLighting |> toEffect ]
        |> Sg.viewTrafo (cameraView |> AVal.map CameraView.viewTrafo)
        |> Sg.projTrafo (frustum |> AVal.map Frustum.projTrafo)


    let renderTask =
        app.Runtime.CompileRender(win.FramebufferSignature, sg)

    win.RenderTask <- renderTask
    win.Run()
    0
