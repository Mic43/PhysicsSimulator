namespace PhysicsSimulator.Entities

open PhysicsSimulator.Utilities

type internal OrientedBox =
    { Faces: Face list
      Vertices: Vector3D list }

module internal OrientedBox =
    let private getOrientedFaces colliderBox rigidBody =
        let getOrientedVertex =
            GraphicsUtils.toWorldCoordinates rigidBody.Variables.Orientation rigidBody.MassCenterPosition

        colliderBox
        |> Box.getFaces
        |> List.map (fun face ->
            { Vertices = face.Vertices |> List.map getOrientedVertex
              Normal =
                face.Normal.Get
                |> (GraphicsUtils.toWorldCoordinates rigidBody.Variables.Orientation Vector3D.zero)
                |> NormalVector.createUnsafe })

    let create box rigidBody =
        let faces = getOrientedFaces box rigidBody 
        let vertices = faces |> List.collect (_.Vertices) |> List.distinct

        { Faces = faces; Vertices = vertices }
    let getEdges orientedBox = 
        orientedBox.Faces |> List.collect Face.getEdges |> List.distinct