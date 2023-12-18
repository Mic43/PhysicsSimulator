namespace PhysicsSimulator

open MathNet.Numerics.LinearAlgebra
open FSharpPlus

module Utils =
    let rec subSetsOf2<'t when 't: comparison> (set: 't Set) =
        if (set.Count < 2) then
            Set.empty
        elif (set.Count = 2) then
            set |> Set.singleton
        else
            let maximumElement = set |> Set.toSeq |> Seq.head
            let subset = set |> Set.remove maximumElement

            subset
            |> subSetsOf2
            |> Set.union (subset |> Set.map (fun el -> [ el; maximumElement ] |> Set.ofList))

    [<TailCall>]
    let rec private subSetsOf2Ag<'t when 't: comparison> (processed: 't Set) rest accumulator =
        if rest |> Set.count = 0 then
            accumulator
        else
            let maximumElement = rest |> Set.toSeq |> Seq.head

            subSetsOf2Ag (processed |> Set.add maximumElement) (rest |> Set.remove maximumElement) accumulator
            |> Set.union (processed |> Set.map (fun el -> [ el; maximumElement ] |> Set.ofList))

    let subSetsOf2Tail set = subSetsOf2Ag Set.empty set Set.empty

module GraphicsUtils =

    // Performs a plane/edge collision test, if an intersection does occur then
    //    it will return the point on the line where it intersected the given plane.
    let tryGetPlaneEdgeIntersection (plane: Plane) (startPoint: Vector3D) (endPoint: Vector3D) : Vector3D option =
        let eps = 1e-6f |> float
        let ab = endPoint.Get() - startPoint.Get()
        //Check that the edge and plane are not parallel and thus never intersect
        // We do this by projecting the line (start - A, End - B) ab along the plane
        let ab_p = plane.Normal.Get().DotProduct(ab)

        if abs (ab_p) > eps then
            //Generate a random point on the plane (any point on the plane will suffice)
            let p_co = plane.Normal.Get() * (-plane.DistanceFromOrigin)
            let fac = -(plane.Normal.Get().DotProduct(startPoint.Get() - p_co)) / ab_p

            //Stop any large floating point divide issues with almost parallel planes
            let fac = min (max fac 0.0) 1.0

            //Return point on edge
            startPoint.Get() + ab * fac |> Vector3D.ofVector |> Some
        else
            None

    let isPointInPlane plane (point: Vector3D) =
        point.Get().DotProduct(plane.Normal.Get()) + plane.DistanceFromOrigin >= 0.0

    let SutherlandHodgmanClipping
        (inputPolygon: Vector3D list)
        (clipPlanes: Plane list)
        (removeNotClipToPlane: bool)
        : Vector3D list =

        let rec loop (inputPolygon: Vector3D list) (planes: Plane list) =
            match planes with
            | [] -> inputPolygon
            | plane :: planesTail ->
                let edges =
                    (inputPolygon |> List.last, inputPolygon |> List.head)
                    :: (inputPolygon |> List.pairwise)

                let clippedPolygon =
                    seq {
                        for startPoint, endPoint in edges do
                            yield!
                                (let isPointInCurPlane = isPointInPlane plane

                                 let getEdgeIntersection startPoint endPoint =
                                     (startPoint, endPoint)
                                     ||> tryGetPlaneEdgeIntersection plane
                                     |> Option.map List.singleton
                                     |> Option.defaultValue []

                                 let iStartIn, isEndIn =
                                     startPoint |> isPointInCurPlane, endPoint |> isPointInCurPlane

                                 match (iStartIn, isEndIn) with
                                 | true, true -> [ endPoint ]
                                 | true, false -> getEdgeIntersection startPoint endPoint
                                 | false, true -> getEdgeIntersection startPoint endPoint @ [ endPoint ]
                                 | false, false -> [])
                    }
                    |> Seq.toList

                loop clippedPolygon planesTail

        if inputPolygon.Length < 3 then
            "polygon has less than 3 vertices" |> invalidArg (nameof (inputPolygon))
        loop inputPolygon clipPlanes
