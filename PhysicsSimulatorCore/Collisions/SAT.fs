namespace PhysicsSimulator.Collisions

open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities

open FSharpPlus

module internal SAT =
    open Vector3D

    /// axes on both boxes that make this edge separating axis
    type EdgeSATAxisData = { EdgesAxes: NormalVector SetOf2 }

    type SATAxisOrigin =
        | Face1 // face of first box
        | Face2
        | Edge of EdgeSATAxisData

    type PossibleCollisionData =
        { NormalFromTarget: NormalVector
          Penetration: float }

    type SATResult =
        { Origin: SATAxisOrigin
          Reference: OrientedBox
          Incident: OrientedBox
          Penetration: float
          CollisionNormalFromReference: NormalVector }

    module SphereBox =
        type SATResult =
            { CollidedFace: Face
              SphereCenter:Vector3D
              Penetration: float
              CollisionNormalFromBox: NormalVector }
        
        let tryFindSeparatingAxis (object1: Sphere * RigidBody) (object2: Box * RigidBody) =
            let orientedBox = object2 ||> OrientedBox.create
            let spherePosition = (object1 |> snd).MassCenterPosition
            let sphereRadius = (object1 |> fst).Radius
            let facesAxes = orientedBox.Faces |> mapWith (_.Normal)

            let tryGetCollisionDataForAxis (axis: NormalVector) : PossibleCollisionData option =
                let vertices = orientedBox.Vertices |> List.map (dotProduct axis.Get)

                let a = vertices |> List.min
                let b = vertices |> List.max

                let c = spherePosition |> dotProduct axis.Get
                let c1 = [c + sphereRadius;c - sphereRadius] |> List.min
                let c2 = [c + sphereRadius;c - sphereRadius] |> List.max

                if c1 >= a && c1 <= b then
                    { NormalFromTarget = axis
                      Penetration = b - c1 }
                    |> Some
                elif  c1 <= a && c2 >= a then
                    { NormalFromTarget = axis
                      Penetration = c2 - a }
                    |> Some
                else
                    None

            let possibleCollisions =
                facesAxes
                |> Seq.map (fun (face, axis) ->
                    axis
                    |> tryGetCollisionDataForAxis
                    |> Option.map (fun collisionData -> (face, collisionData)))

            possibleCollisions
            |> sequence
            |> Option.map (
                Seq.minBy (fun (_, cd) -> cd.Penetration)
                >> (fun (face, cd) ->
                    { CollidedFace = face
                      Penetration = cd.Penetration
                      CollisionNormalFromBox = cd.NormalFromTarget
                      SphereCenter = spherePosition })
            )


    // axis must correspond to one of the target's vertices
    let private tryGetCollisionDataForAxis polyhedrons (axis: NormalVector) : PossibleCollisionData option =
        let withProjection (vertices: Vector3D seq) =
            vertices |> Seq.map (fun v -> (v, v |> dotProduct axis.Get))

        let vertices = polyhedrons |> SetOf2.map (_.Vertices >> withProjection)

        let minMax =
            vertices
            |> SetOf2.map (fun s ->
                {| Min = s |> Seq.minBy snd
                   Max = s |> Seq.maxBy snd |})

        let _, a = (minMax |> SetOf2.fst).Min
        let _, b = (minMax |> SetOf2.fst).Max
        let _, c = (minMax |> SetOf2.snd).Min
        let _, d = (minMax |> SetOf2.snd).Max

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

    let private chooseSeparationAxis axes =
        monad' {
            let! bestAxes = axes |> sequence

            let origin, (collisionData: PossibleCollisionData), boxes =
                bestAxes |> Seq.minBy (fun (_, cd, _) -> cd.Penetration)

            return
                { Origin = origin
                  Reference = boxes |> SetOf2.fst
                  Incident = boxes |> SetOf2.snd
                  CollisionNormalFromReference = collisionData.NormalFromTarget
                  Penetration = collisionData.Penetration }
        }

    let private getPossibleCollisions axesWithData =
        axesWithData
        |> Seq.map (fun (origin, boxes, axis) ->
            tryGetCollisionDataForAxis boxes axis
            |> Option.map (fun collisionData -> (origin, collisionData, boxes)))

    let tryFindSeparatingAxis (objects: (Box * RigidBody) SetOf2) : SATResult option =
        let bodies = objects |> SetOf2.map snd
        let orientedBoxes = objects |> SetOf2.map (fun p -> p ||> OrientedBox.create)

        let fromFirst =
            (bodies |> SetOf2.snd).MassCenterPosition
            - (bodies |> SetOf2.fst).MassCenterPosition

        let facesAxes =
            orientedBoxes |> SetOf2.map _.Faces |> SetOf2.map (List.map (_.Normal))

        let edgeAxes =
            seq {
                for firstAxis in bodies |> SetOf2.fst |> RigidBody.getAxes do
                    for secondAxis in bodies |> SetOf2.snd |> RigidBody.getAxes do
                        yield
                            ([ firstAxis; secondAxis ] |> SetOf2.ofList,
                             firstAxis.Get |> crossProduct secondAxis.Get |> normalized)
            }
            |> Seq.filter (fun (_, axis) -> axis.Get |> isZero 0.01 |> not)
            |> Seq.map (Tuple2.mapItem2 (fun axis -> if fromFirst |> dotProduct axis.Get < 0 then -axis else axis))

        let axesWithData =
            seq {
                for axis in facesAxes |> SetOf2.fst do
                    yield (Face1, orientedBoxes, axis)

                for axis in facesAxes |> SetOf2.snd do
                    yield (Face2, orientedBoxes |> SetOf2.flip, axis)

                for edge, edgeSepAxis in edgeAxes do
                    yield ({ EdgeSATAxisData.EdgesAxes = edge } |> Edge, orientedBoxes, edgeSepAxis)
            }

        axesWithData |> getPossibleCollisions |> chooseSeparationAxis
