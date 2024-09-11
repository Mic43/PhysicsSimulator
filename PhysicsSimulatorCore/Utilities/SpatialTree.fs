namespace PhysicsSimulator.Utilities

open FSharpPlus


type SpatialTreeNode<'T> =
    | Leaf of 'T list
    | NonLeaf of SpatialTreeNode<'T> array

/// n dimensional spatial tree
type SpatialTree<'T> =
    { Root: SpatialTreeNode<'T>
      MaxLeafObjects: int
      MaxDepth: int
      SpaceBoundaries: (float * float) list }

type ObjectExtent = { Size: float; Position: float }

type private NodeData =
    { Sizes: float list
      Positions: float list }

    static member init tree =
        { Sizes = tree.SpaceBoundaries |> List.map (fun (a, b) -> b - a)
          Positions = tree.SpaceBoundaries |> List.map fst }

module SpatialTree =
    let private failIfWrongDimension tree list =
        if (tree.SpaceBoundaries |> List.length) <> (list |> List.length) then
            "wrong object extent dimension" |> invalidArg "objectExtentProvider"

    let private getChildNodeIndexByPosition tree (position: float list) =
        do position |> failIfWrongDimension tree

        let boundaries = tree.SpaceBoundaries |> Seq.ofList
        let middles = boundaries |> Seq.map (fun (a, b) -> (a + b) / 2.0)

        let notValid bounds coord =
            coord < (bounds |> fst) || coord > (bounds |> snd)

        let leftRight =
            (position |> Seq.ofList, middles, boundaries)
            |||> Seq.zip3
            |> Seq.map (fun (coord, middle, bounds) ->
                if coord |> notValid bounds then
                    "position outside of bounds" |> invalidArg "position"

                if coord < middle then 0 else 1)

        leftRight
        |> Seq.indexed
        |> Seq.fold (fun acc (i, cur) -> acc + cur * (pown 2 i)) 0

    let private getNodePositionByIndex index curNodeSize =
        (index, [])
        |> List.foldBack
            (fun curSize (indexRest, pos) ->
                let v = if indexRest % 2 = 0 then 0.0 else (curSize / 2.0)
                (indexRest / 2, v :: pos))
            curNodeSize
        |> snd

    let private getChildNodeData parentNodeData index =
        let childNodePosition =
            getNodePositionByIndex index parentNodeData.Sizes
            |> List.zip parentNodeData.Positions
            |> List.map (fun (pos, size) -> pos + size)

        { Positions = childNodePosition
          Sizes = parentNodeData.Sizes |> List.map (fun s -> s / 2.0) }

    let private areIntersecting nodeDataA nodeDataB =
        let size =
            nodeDataA.Sizes
            |> Seq.ofList
            |> Seq.zip nodeDataB.Sizes
            |> Seq.map (fun (s1, s2) -> s2 + s1)

        let delta =
            nodeDataA.Positions
            |> Seq.ofList
            |> Seq.zip nodeDataB.Positions
            |> Seq.map (fun (p1, p2) -> p2 - p1)

        Seq.forall2 (fun delta size -> abs delta < size) delta size

    let init<'T> maxLeafObjects maxDepth spaceBoundaries : SpatialTree<'T> =
        if spaceBoundaries |> List.isEmpty then
            "boundaries must be non empty" |> invalidArg "spaceBoundaries"

        if maxLeafObjects < 0 then
            "must be positive" |> invalidArg "maxLeafObjects"

        if maxDepth < 0 then
            "must be positive" |> invalidArg "maxDepth"

        { Root = [] |> Leaf
          MaxLeafObjects = maxLeafObjects
          MaxDepth = maxDepth
          SpaceBoundaries = spaceBoundaries }

    let insert
        (tree: SpatialTree<'T>)
        (objectExtentProvider: 'T -> ObjectExtent list)
        (object: 'T)
        =
        let objectExtent = object |> objectExtentProvider
        do objectExtent |> failIfWrongDimension tree

        let rec addInternal curDepth (curNodeData: NodeData) (node: SpatialTreeNode<'T>) : SpatialTreeNode<'T> =

            let toNodeData (extent: ObjectExtent list) =
                let p = extent |> List.map (fun e -> (e.Size, e.Position)) |> List.unzip
                { Sizes = p |> fst; Positions = p |> snd }

            let splitNode nodeObjects =
                [| 0 .. (pown 2 tree.SpaceBoundaries.Length) - 1 |]
                |> Array.map (fun index ->
                    nodeObjects
                    |> List.filter (fun nodeObject ->
                        index
                        |> getChildNodeData curNodeData
                        |> areIntersecting (nodeObject |> objectExtentProvider |> toNodeData))
                    |> Leaf)
                |> NonLeaf

            match node with
            | Leaf objects ->
                if (objects |> List.length) < tree.MaxLeafObjects || (curDepth >= tree.MaxDepth) then
                    object :: objects |> Leaf
                else
                    object :: objects |> splitNode
            | NonLeaf childNodes ->
                childNodes
                |> Array.mapi (fun i childNode ->
                    let childNodeData = i |> getChildNodeData curNodeData

                    if childNodeData |> areIntersecting (objectExtent |> toNodeData) then
                        childNode |> addInternal (curDepth + 1) childNodeData
                    else
                        childNode)
                |> NonLeaf

        { tree with
            Root = tree.Root |> addInternal 1 (tree |> NodeData.init) }

    let getObjectBuckets (tree: SpatialTree<'T>) =
        let rec getBucketsInternal node : 'T list seq =
            match node with
            | Leaf objects when objects.IsEmpty -> Seq.empty
            | Leaf objects -> objects |> Seq.singleton
            | NonLeaf spatialTreeNodes -> spatialTreeNodes |> Array.toSeq |> Seq.collect getBucketsInternal

        getBucketsInternal tree.Root
