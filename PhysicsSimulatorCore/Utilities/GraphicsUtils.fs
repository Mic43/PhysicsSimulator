namespace PhysicsSimulator.Utilities

open MathNet.Numerics.LinearAlgebra
open FSharpPlus

module GraphicsUtils =
    open Vector3D

    let getClosestPointTo (vertex1: Vector3D, vertex2: Vector3D) position =
        let diff_AP = position - vertex1
        let diff_AB = vertex2 - vertex1

        //Distance along the line of point 'pos' in world distance
        let ABAPproduct = diff_AP |> dotProduct diff_AB
        let magnitudeAB = diff_AB |> dotProduct diff_AB

        //Distance along the line of point 'pos' between 0-1 where 0 is line start and 1 is line end
        let distance = ABAPproduct / magnitudeAB

        //Clamp distance so it cant go beyond the line's start/end in either direction
        let distance = (distance |> min 1.0) |> max 0.0

        //Use distance from line start (0-1) to compute the actual position
        vertex1 + diff_AB * distance

    let getClosestPointToPoly (position: Vector3D) (polygon: Vector3D list) =
        let edges =
            (polygon |> List.last, polygon |> List.head) :: (polygon |> List.pairwise)

        seq {
            for startPoint, endPoint in edges do
                let edgeClosestPoint = position |> getClosestPointTo (startPoint, endPoint)
                let diff = position - edgeClosestPoint
                yield (diff |> dotProduct diff, edgeClosestPoint)
        }
        |> Seq.minBy fst
        |> snd

    // Performs a plane/edge collision test, if an intersection does occur then
    //    it will return the point on the line where it intersected the given plane.
    let tryGetPlaneEdgeIntersection
        epsilon
        (plane: Plane)
        (startPoint: Vector3D)
        (endPoint: Vector3D)
        : Vector3D option =
        let ab = endPoint - startPoint
        //Check that the edge and plane are not parallel and thus never intersect
        // We do this by projecting the line (start - A, End - B) ab along the plane
        let ab_p = plane.Normal.Get |> dotProduct (ab)

        if abs ab_p > epsilon then
            //Generate a random point on the plane (any point on the plane will suffice)
            let p_co = plane.Normal.Get * (plane.DistanceFromOrigin)
            let fac = -(plane.Normal.Get |> dotProduct (startPoint - p_co)) / ab_p

            //Stop any large floating point divide issues with almost parallel planes
            let fac = min (max fac 0.0) 1.0

            //Return point on edge
            startPoint + ab * fac |> Some
        else
            None

    let isPointInPlane plane (point: Vector3D) =
        (point |> dotProduct plane.Normal.Get) - plane.DistanceFromOrigin >= 0.0

    /// Planes normals must point to polygon parts that should be left after clipping
    let SutherlandHodgmanClipping epsilon (clipPlanes: Plane list) (inputPolygon: Vector3D list) : Vector3D list =

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
                                     ||> tryGetPlaneEdgeIntersection epsilon plane
                                     |> Option.map List.singleton
                                     |> Option.defaultValue []

                                 let iStartIn, isEndIn =
                                     startPoint |> isPointInCurPlane, endPoint |> isPointInCurPlane

                                 match (iStartIn, isEndIn) with
                                 | true, true -> [ endPoint ]
                                 | true, false -> getEdgeIntersection startPoint endPoint
                                 //TODO: replace @ with prepending
                                 | false, true -> getEdgeIntersection startPoint endPoint @ [ endPoint ]
                                 | false, false -> [])
                    }
                    |> Seq.toList

                loop clippedPolygon planesTail

        loop inputPolygon clipPlanes |> List.distinct

    let toWorldCoordinates (rotationMatrix: Matrix3) (translationVector: Vector3D) (v: Vector3D) =
        v
        |> ((fun v -> rotationMatrix.Get * v.Get) >> (fun v -> v + translationVector.Get))
        |> ofVector
    
