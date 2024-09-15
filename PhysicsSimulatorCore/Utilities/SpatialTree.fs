namespace PhysicsSimulator.Utilities

open FSharpPlus


type SpatialTreeNode<'T> =
    | Leaf of 'T list // TODO: maybe store set instead of list?
    | NonLeaf of SpatialTreeNode<'T> array

/// n dimensional spatial tree
type SpatialTree<'T> =
    { Root: SpatialTreeNode<'T>
      MaxLeafObjects: int
      MaxDepth: int
      SpaceBoundaries: (float * float) list }

///Position (left top) and size of the boundingBox 
type ObjectExtent =
    { Size: float
      Position: float }

    member this.withPositionCentered =
        { this with
            Position = this.Position + this.Size / 2.0 }

type private NodeData =
    | NodeData of ObjectExtent list

    static member init tree =
        tree.SpaceBoundaries
        |> List.map (fun (min, max) -> { Size = max - min; Position = min })
        |> NodeData

module SpatialTree =    
    let private areIntersecting (NodeData nodeDataA) (NodeData nodeDataB) =
        (nodeDataA |> List.map _.withPositionCentered, nodeDataB |> List.map _.withPositionCentered)
        ||> List.forall2 (fun ndA ndB -> (ndA.Size + ndB.Size) / 2.0 >= abs (ndB.Position - ndA.Position))
    
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

    let private getChildNodeData (NodeData parentNodeData) index =
        let childNodePositionAbsolute =
            getNodePositionByIndex index (parentNodeData |> List.map (_.Size))

        (parentNodeData |> List.map (fun oe -> { oe with Size = oe.Size / 2.0 }), childNodePositionAbsolute)
        ||> List.map2 (fun parent child ->
            { parent with
                Position = parent.Position + child })
        |> NodeData

    let init<'T> maxLeafObjects maxDepth spaceBoundaries : SpatialTree<'T> =
        if spaceBoundaries |> List.isEmpty then
            "boundaries must be non empty" |> invalidArg "spaceBoundaries"

        if maxLeafObjects < 1 then
            "must be positive" |> invalidArg "maxLeafObjects"

        if maxDepth < 1 then
            "must be positive" |> invalidArg "maxDepth"

        { Root = [] |> Leaf
          MaxLeafObjects = maxLeafObjects
          MaxDepth = maxDepth
          SpaceBoundaries = spaceBoundaries }

    let insert (tree: SpatialTree<'T>) (objectExtentProvider: 'T -> ObjectExtent list) (object: 'T) =
        let objectExtent = object |> objectExtentProvider
        
        do objectExtent |> failIfWrongDimension tree
        if tree |> NodeData.init |> areIntersecting (objectExtent |> NodeData) |> not then
            invalidArg "object" $"object {object} out of space bounds"
        
        let rec addInternal curDepth (curNodeData: NodeData) (node: SpatialTreeNode<'T>) : SpatialTreeNode<'T> =
            let splitNode nodeObjects =
                [| 0 .. (pown 2 tree.SpaceBoundaries.Length) - 1 |]
                |> Array.map (fun index ->
                    nodeObjects
                    |> List.filter (fun nodeObject ->
                        index
                        |> getChildNodeData curNodeData
                        |> areIntersecting (nodeObject |> objectExtentProvider |> NodeData))
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

                    if childNodeData |> areIntersecting (objectExtent |> NodeData) then
                        childNode |> addInternal (curDepth + 1) childNodeData
                    else
                        childNode)
                |> NonLeaf

        { tree with
            Root = tree.Root |> addInternal 1 (tree |> NodeData.init) }

    //TODO: should be seq of sets?
    let getObjectBuckets (tree: SpatialTree<'T>) =
        let rec getBucketsInternal node : 'T list seq =
            match node with
            | Leaf objects when objects.IsEmpty -> Seq.empty
            | Leaf objects -> objects |> Seq.singleton
            | NonLeaf spatialTreeNodes -> spatialTreeNodes |> Array.toSeq |> Seq.collect getBucketsInternal

        getBucketsInternal tree.Root
