module Gui.HelpOverlay

open System.Numerics
open Aardvark.Rendering.ImGui
open Hexa.NET.ImGui

let private commonHelpLines =
    [| "Sterowanie (wspolne)"
       "Pauza - pauza / wznowienie"
       "R - reset sceny"
       "C - kolizje (wl./wyl.)"
       "Mysz - kamera" |]

let create (win: Aardvark.Glfw.Window) (sceneLines: string list) =
    let gui = win.InitializeImGui()

    gui.Render <-
        fun () ->
            ImGui.SetNextWindowPos(Vector2(12.0f, 12.0f), ImGuiCond.Always)
            ImGui.SetNextWindowBgAlpha(0.82f)

            let flags =
                ImGuiWindowFlags.NoDecoration
                ||| ImGuiWindowFlags.NoMove
                ||| ImGuiWindowFlags.NoSavedSettings
                ||| ImGuiWindowFlags.AlwaysAutoResize

            if ImGui.Begin("Sterowanie", flags) then
                for line in commonHelpLines do
                    ImGui.Text(line)

                if not sceneLines.IsEmpty then
                    ImGui.Separator()

                    for line in sceneLines do
                        ImGui.Text(line)

                ImGui.End()

    gui
