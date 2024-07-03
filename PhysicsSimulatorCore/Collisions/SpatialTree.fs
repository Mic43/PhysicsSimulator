namespace PhysicsSimulator.Collisions

open FSharpPlus


type SpatialTreeNode<'T> =
    | Leaf of 'T list
    | NonLeaf of SpatialTreeNode<'T> array

/// n dimensional spatial tree
type SpatialTree<'T> =
    private
        { Root: SpatialTreeNode<'T>
          MaxLeafObjects: int
          MaxDepth: int
          SpaceBoundaries: (float * float) list }

type private NodeData =
    { Size: float list
      Position: float list }

    static member init tree =
        { Size = (tree.SpaceBoundaries |> List.map (fun (a, b) -> b - a))
          Position = List.init tree.SpaceBoundaries.Length (fun _ -> 0.0) }

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
        curNodeSize
        |> List.fold
            (fun (rest, pos) cur ->
                if rest % 2 = 0 then
                    (rest / 2, 0.0 :: pos)
                else
                    (rest / 2, (cur / 2.0) :: pos))
            (index, [])
        |> snd
    // curNodeSize |> List.map (fun s -> )
    // let mutable i = index
    // while i > 0 do
    //     let r = i % 2
    //     i <- i / 2
    // ()
    let create maxLeafObjects maxDepth spaceBoundaries =
        if spaceBoundaries |> List.isEmpty then
            "boundaries must be non empty" |> failwith "spaceBoundaries"

        { Root = [] |> Leaf
          MaxLeafObjects = maxLeafObjects
          MaxDepth = maxDepth
          SpaceBoundaries = spaceBoundaries }

    // let add (tree: SpatialTree<'T>) (objectExtentProvider: 'T -> (float * float) list) (object: 'T) =
    //     let extent = object |> objectExtentProvider
    //
    //     do extent |> failIfWrongDimension tree
    //
    //     let rec addInternal curDepth (curNodeData: NodeData) (node: SpatialTreeNode<'T>) : SpatialTreeNode<'T> =
    //         match node with
    //         | Leaf objects ->
    //             if (objects |> List.length) < tree.MaxLeafObjects || (curDepth >= tree.MaxDepth) then
    //                 object :: objects |> Leaf
    //             else
    //                 objects |> splitNode
    //         | NonLeaf childNodes ->
    //             childNodes
    //             |> Array.mapi (fun i childNode ->
    //                 let childPosition =
    //                     getNodePositionByIndex i curNodeData.Size
    //                     |> List.map ((+) curNodeData.Position[i])
    //
    //                 if childNode |> isIntersecting object then
    //                     childNode
    //                     |> addInternal
    //                         (curDepth + 1)
    //                         { curNodeData with
    //                             Size = curNodeData.Size |> List.map (fun s -> s / 2.0) }                  
    //                 else
    //                     childNode)
    //             |> NonLeaf
    //
    //     tree.Root |> addInternal 0 (tree |> NodeData.init)
