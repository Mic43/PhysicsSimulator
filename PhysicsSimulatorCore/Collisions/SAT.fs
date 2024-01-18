namespace PhysicsSimulator.Collisions

open FSharpPlus.Internals
open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities

open MathNet.Numerics.LinearAlgebra
open FSharpPlus
open FSharpPlus.Data

module SAT =
    open Vector3D

    type SATAxisOrigin =
        | Faces1
        | Faces2
        | Edges

    type Polyhedron =
        { Faces: Face seq
          Vertices: Vector3D seq }

    type PossibleCollisionData =
        { NormalFromTarget: Vector3D
          Penetration: float }

    type SATResult =
        { Origin: SATAxisOrigin
          Reference: Polyhedron
          Incident: Polyhedron
          CollisionNormalFromReference: Vector3D }

    let private chooseSeparationAxis candidatesMap =
        let origin, collisionData, polyhedrons =
            candidatesMap
            |> Map.toList
            |> List.map (fun (axis, (cd, polyhedron)) -> (axis, cd, polyhedron))
            |> List.minBy (fun (_, cd, _) -> cd.Penetration)

        { Origin = origin
          Reference = polyhedrons |> SetOf2.fst
          Incident = polyhedrons |> SetOf2.snd
          CollisionNormalFromReference = collisionData.NormalFromTarget }
    // axis must correspond to one if the target's vertices
    
    let private tryGetCollisionDataForAxis
        (polyhedronTarget: Polyhedron)
        (polyhedronOther: Polyhedron)
        (axis: Vector3D)
        : PossibleCollisionData option =
        let withProjection (vertices: Vector3D seq) =
            vertices |> Seq.map (fun v -> (v, v |> dotProduct axis))

        let v1 = polyhedronTarget.Vertices |> withProjection
        let v2 = polyhedronOther.Vertices |> withProjection

        let minMax =
            [ v1; v2 ]
            |> List.map (fun s ->
                {| Min = s |> Seq.minBy snd
                   Max = s |> Seq.maxBy snd |})

        let min1, a = minMax[0].Min
        let max1, b = minMax[0].Max
        let min2, c = minMax[1].Min
        let max2, d = minMax[1].Max

        // a    c  b           d
        if a <= c && b >= c then
            { NormalFromTarget = axis
              Penetration = b - c }
            |> Some
        // c     a     d  b
        elif c <= a && d >= a then
            { NormalFromTarget = -axis
              Penetration = d - a }
            |> Some
        else
            None

    let private getOrientedFaces (colliderBox, rigidBody: RigidBody) =
        let getOrientedVertex =
            GraphicsUtils.toWorldCoordinates rigidBody.Variables.Orientation rigidBody.MassCenter.Variables.Position

        colliderBox
        |> Collider.getFaces
        |> Seq.map (fun face ->
            { Vertices = face.Vertices |> Seq.map getOrientedVertex
              Normal =
                face.Normal
                |> GraphicsUtils.toWorldCoordinates rigidBody.Variables.Orientation zero })

    let tryFindSeparatingAxis (objects: (Box * RigidBody) SetOf2) : SATResult option =

        let faces = objects |> SetOf2.map getOrientedFaces

        let vertices =
            faces |> SetOf2.map (Seq.bind (fun face -> face.Vertices |> Seq.distinct))

        let polyhedrons =
            (faces, vertices)
            ||> SetOf2.zip
            |> SetOf2.map (fun (f, v) -> { Faces = f; Vertices = v })

        let facesAxes = faces |> SetOf2.map (Seq.map (_.Normal))

        let edgesAxes =
            facesAxes
            |> SetOf2.snd
            |> Seq.apply (facesAxes |> SetOf2.fst |> Seq.map crossProduct)
            |> Seq.map (fun v -> v |> normalized)

        let axesWithData =
            [ (Faces1, (polyhedrons, facesAxes |> SetOf2.fst))
              (Faces2, (polyhedrons |> SetOf2.flip, facesAxes |> SetOf2.snd))
              //TODO: finish fo edges
              //(Edges, (polyhedrons, edgesAxes))
              ]
            |> Map.ofList

        let bestAxes: Map<SATAxisOrigin, (PossibleCollisionData * SetOf2<Polyhedron>) option> =
            axesWithData
            |> Map.mapValues (
                (fun (polyhedrons, axes) ->
                    axes
                    |> Seq.map (
                        tryGetCollisionDataForAxis (polyhedrons |> SetOf2.fst) (polyhedrons |> SetOf2.snd)
                        >> Option.map (fun cd -> (cd, polyhedrons))
                    ))
                >> (Seq.sequence >> Option.map (Seq.minBy (fun (cd, _) -> cd.Penetration)))
            )

        //TODO: improve
        if bestAxes |> Map.exists (fun _ axis -> axis.IsNone) then
            None
        else
            bestAxes |> Map.mapValues Option.get |> chooseSeparationAxis |> Some
