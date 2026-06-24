module Gui.PhongLighting

open Aardvark.Base
open Aardvark.Rendering
open Aardvark.Rendering.Effects
open Aardvark.SceneGraph
open FSharp.Data.Adaptive
open FShade

/// Phong without texture uniforms (DefaultSurfaces.lighting requires SpecularColorTexture).
let private phongTwoSided (v : Vertex) =
    fragment {
        let n = v.n |> Vec.normalize
        let l = (uniform.LightLocation - v.wp.XYZ) |> Vec.normalize
        let ndotl = Vec.dot l n |> abs

        let ambient = 0.15f
        let diffuse = ambient + (1.0f - ambient) * ndotl
        let spec = (Vec.dot l n |> abs) ** 32.0f

        return V4f(v.c.XYZ * diffuse + V3f.III * 0.3f * spec, v.c.W)
    }

let apply (lightLocation : aval<V3d>) (sg : ISg) =
    sg
    |> Sg.uniform "LightLocation" lightLocation
    |> Sg.effect [ DefaultSurfaces.trafo |> toEffect; phongTwoSided |> toEffect ]
