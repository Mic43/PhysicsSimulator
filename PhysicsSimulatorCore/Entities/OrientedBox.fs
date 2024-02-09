namespace PhysicsSimulator.Entities

open PhysicsSimulator.Utilities

type OrientedBox =
    { Faces: Face list
      Vertices: Vector3D list }


module OrientedBox =
    let private getOrientedFaces colliderBox rigidBody =
        let getOrientedVertex =
            GraphicsUtils.toWorldCoordinates rigidBody.Variables.Orientation rigidBody.MassCenterPosition

        colliderBox
        |> Box.getFaces
        |> Seq.map (fun face ->
            { Vertices = face.Vertices |> List.map getOrientedVertex
              Normal =
                face.Normal.Get
                |> (GraphicsUtils.toWorldCoordinates rigidBody.Variables.Orientation Vector3D.zero)
                |> NormalVector.createUnsafe })

    let create box rigidBody =
        let faces = getOrientedFaces box rigidBody |> List.ofSeq
        let vertices = faces |> List.collect (fun face -> face.Vertices |> List.distinct)

        { Faces = faces; Vertices = vertices }
    let getEdges orientedBox = 
        orientedBox.Faces |> List.collect Face.getEdges |> List.distinct