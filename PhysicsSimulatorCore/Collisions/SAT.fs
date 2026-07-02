namespace PhysicsSimulator.Collisions

open PhysicsSimulator.Utilities
open PhysicsSimulator.Entities

open FSharpPlus

module internal SAT =
    open Vector3D

    /// axes on both boxes that make this edge separating axis
    type EdgeSATAxisData = { EdgesAxes: NormalVector Pair }

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

        type private AxisOverlapTestResult =
            | SeparatedOnAxis
            | OverlappingOnAxis of overlap: float * normalFromBox: Vector3D

        let private tryGetOverlapOnAxis
            spherePosition
            sphereRadius
            boxCenter
            (boxAxis: NormalVector)
            halfExtent
            : AxisOverlapTestResult =
            let axis = boxAxis.Get
            let boxProjection = boxCenter |> dotProduct axis
            let boxMin = boxProjection - halfExtent
            let boxMax = boxProjection + halfExtent

            let sphereProjection = spherePosition |> dotProduct axis
            let sphereMin = sphereProjection - sphereRadius
            let sphereMax = sphereProjection + sphereRadius

            if sphereMax < boxMin || sphereMin > boxMax then
                SeparatedOnAxis
            else
                let overlap = (min boxMax sphereMax) - (max boxMin sphereMin)
                let relativePosition = sphereProjection - boxProjection

                let normalFromBox =
                    if relativePosition >= 0.0 then axis else -axis

                OverlappingOnAxis(overlap, normalFromBox)

        let private findCollidedFace (orientedBox: OrientedBox) (normalFromBox: Vector3D) =
            orientedBox.Faces
            |> List.maxBy (fun face -> face.Normal.Get |> dotProduct normalFromBox)

        /// SAT on the three principal box axes: returns None when any axis separates the shapes.
        /// Overlap on all three axes is necessary but not sufficient for a true sphere-box hit (corner case).
        let tryFindSeparatingAxisSAT (object1: Sphere * RigidBody) (object2: Box * RigidBody) =
            let boxCollider, boxBody = object2
            let orientedBox = object2 ||> OrientedBox.create
            let spherePosition = (object1 |> snd).MassCenterPosition
            let sphereRadius = (object1 |> fst).Radius
            let boxCenter = boxBody.MassCenterPosition

            let halfExtents =
                [ boxCollider.XSize / 2.0
                  boxCollider.YSize / 2.0
                  boxCollider.ZSize / 2.0 ]

            let axisTests =
                (boxBody |> RigidBody.getAxes, halfExtents)
                ||> List.map2 (tryGetOverlapOnAxis spherePosition sphereRadius boxCenter)

            if axisTests |> List.exists ((=) SeparatedOnAxis) then
                None
            else
                let overlap, normalFromBox =
                    axisTests
                    |> List.choose (function
                        | OverlappingOnAxis(overlap, normalFromBox) -> Some(overlap, normalFromBox)
                        | SeparatedOnAxis -> None)
                    |> List.minBy fst

                let collidedFace = findCollidedFace orientedBox normalFromBox

                { CollidedFace = collidedFace
                  Penetration = overlap
                  CollisionNormalFromBox = normalFromBox |> NormalVector.createUnsafe
                  SphereCenter = spherePosition }
                |> Some

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

        let vertices = polyhedrons |> Pair.map (_.Vertices >> withProjection)

        let minMax =
            vertices
            |> Pair.map (fun s ->
                {| Min = s |> Seq.minBy snd
                   Max = s |> Seq.maxBy snd |})

        let _, a = (minMax |> Pair.fst).Min
        let _, b = (minMax |> Pair.fst).Max
        let _, c = (minMax |> Pair.snd).Min
        let _, d = (minMax |> Pair.snd).Max

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
                  Reference = boxes |> Pair.fst
                  Incident = boxes |> Pair.snd
                  CollisionNormalFromReference = collisionData.NormalFromTarget
                  Penetration = collisionData.Penetration }
        }

    let private getPossibleCollisions axesWithData =
        axesWithData
        |> Seq.map (fun (origin, boxes, axis) ->
            tryGetCollisionDataForAxis boxes axis
            |> Option.map (fun collisionData -> (origin, collisionData, boxes)))

    let tryFindSeparatingAxis (objects: (Box * RigidBody) Pair) : SATResult option =
        let bodies = objects |> Pair.map snd
        let orientedBoxes = objects |> Pair.map (fun p -> p ||> OrientedBox.create)

        let fromFirst =
            (bodies |> Pair.snd).MassCenterPosition
            - (bodies |> Pair.fst).MassCenterPosition

        let facesAxes =
            orientedBoxes |> Pair.map _.Faces |> Pair.map (List.map (_.Normal))

        let edgeAxes =
            seq {
                for firstAxis in bodies |> Pair.fst |> RigidBody.getAxes do
                    for secondAxis in bodies |> Pair.snd |> RigidBody.getAxes do
                        yield
                            ([ firstAxis; secondAxis ] |> Pair.ofList,
                             firstAxis.Get |> crossProduct secondAxis.Get |> normalized)
            }
            |> Seq.filter (fun (_, axis) -> axis.Get |> isZero 0.01 |> not)
            |> Seq.map (Tuple2.mapItem2 (fun axis -> if fromFirst |> dotProduct axis.Get < 0 then -axis else axis))

        let axesWithData =
            seq {
                for axis in facesAxes |> Pair.fst do
                    yield (Face1, orientedBoxes, axis)

                for axis in facesAxes |> Pair.snd do
                    yield (Face2, orientedBoxes |> Pair.flip, axis)

                for edge, edgeSepAxis in edgeAxes do
                    yield ({ EdgeSATAxisData.EdgesAxes = edge } |> Edge, orientedBoxes, edgeSepAxis)
            }

        axesWithData |> getPossibleCollisions |> chooseSeparationAxis
