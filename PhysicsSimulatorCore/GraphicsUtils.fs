namespace PhysicsSimulator

open MathNet.Numerics.LinearAlgebra
open FSharpPlus
open Constants

module GraphicsUtils =

    // Performs a plane/edge collision test, if an intersection does occur then
    //    it will return the point on the line where it intersected the given plane.
    let tryGetPlaneEdgeIntersection (plane: Plane) (startPoint: Vector3D) (endPoint: Vector3D) : Vector3D option =
        let ab = endPoint.Get - startPoint.Get
        //Check that the edge and plane are not parallel and thus never intersect
        // We do this by projecting the line (start - A, End - B) ab along the plane
        let ab_p = plane.Normal.Get.DotProduct(ab)

        if abs ab_p > epsilon then
            //Generate a random point on the plane (any point on the plane will suffice)
            let p_co = plane.Normal.Get * (-plane.DistanceFromOrigin)
            let fac = -(plane.Normal.Get.DotProduct(startPoint.Get - p_co)) / ab_p

            //Stop any large floating point divide issues with almost parallel planes
            let fac = min (max fac 0.0) 1.0

            //Return point on edge
            startPoint.Get + ab * fac |> Vector3D.ofVector |> Some
        else
            None

    let isPointInPlane plane (point: Vector3D) =
        point.Get.DotProduct(plane.Normal.Get) + plane.DistanceFromOrigin >= 0.0

    /// Planes normals must point to polygon parts that should be left after clipping
    let SutherlandHodgmanClipping
        (clipPlanes: Plane list)
        (inputPolygon: Vector3D list)
        //(removeNotClipToPlane: bool)
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

        loop inputPolygon clipPlanes |> List.distinct
