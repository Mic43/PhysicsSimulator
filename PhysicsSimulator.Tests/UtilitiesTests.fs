module PhysicsSimulator.Tests.UtilitiesTests

open System
open FSharpPlus
open FSharpPlus.Data
open PhysicsSimulator.Tests.Generators.SpatialTree
open Xunit
open FsCheck
open FsCheck.Xunit
open PhysicsSimulator.Utilities

let (.=.) left right = left = right |@ $"%A{left} = %A{right}"

let internal singleNodeTreeFromPosMap spaceBoundaries posMap =
    let count = (posMap |> Map.count) - 1

    { SpatialTree.init<int> count 10 spaceBoundaries with
        Root = Leaf(posMap |> Map.keys |> Seq.take count |> Set.ofSeq) }

module SpatialTree =

    open PhysicsSimulator.Tests

    [<Property>]
    let ``init called with empty boundaries throws exception`` (maxLeafObjects: PositiveInt) (maxDepth: PositiveInt) =
        lazy ((maxLeafObjects.Get, maxDepth.Get, List.Empty) |||> SpatialTree.init)
        |> Prop.throws<ArgumentException, _>

    [<Property>]
    let `` init returns empty tree ``
        (maxLeafObjects: PositiveInt)
        (maxDepth: PositiveInt)
        (spaceBoundaries: NonEmptyList<float * float>)
        =
        let actual =
            (maxLeafObjects.Get, maxDepth.Get, spaceBoundaries |> NonEmptyList.toList)
            |||> SpatialTree.init

        match actual.Root with
        | Leaf l -> l = Set.empty
        | _ -> false

    [<Property>]
    let ``For empty tree getObjectBuckets returns empty seq`` () =

        (fun tree ->
            let actual = tree |> SpatialTree.getObjectBuckets
            let expected = Seq.empty
            actual .=. expected)
        |> Prop.forAll (emptyTree 1000.0 |> Arb.fromGen)

    [<Property>]
    let ``For single object tree getObjectBuckets returns object's singleton `` (object: int) =

        let treeGen =
            gen {
                let size = 1000.0
                let! emptyTree = Generators.SpatialTree.emptyTree<int> size

                let! extent = emptyTree.SpaceBoundaries.Length |> Generators.SpatialTree.extent size

                return
                    object
                    |> SpatialTree.insert emptyTree (fun _ ->
                        emptyTree.SpaceBoundaries |> List.map (fun (p, s) -> { Position = p; Size = s }))
            }

        (fun tree ->
            let actual = tree |> SpatialTree.getObjectBuckets
            let expected = object |> Set.singleton |> Seq.singleton

            actual |> Seq.toList .=. (expected |> Seq.toList))
        |> Prop.forAll (treeGen |> Arb.fromGen)

    [<Property(EndSize = 10)>]
    let ``For tree with full Root node, inserting object causes node to split correctly if maxDepth is not reached``
        object
        =
        let maxSurfaceSize = 10.0

        let treeGen =
            Arb.generate<int>
            |> Gen.filter (fun node -> node <> object)
            |> singleNodeTree maxSurfaceSize
            |> Gen.filter (fun tree -> tree.MaxDepth > 1)

        (fun tree ->
            // all objects are placed in the plane origin
            let actual =
                object
                |> SpatialTree.insert tree (fun _ ->
                    tree.SpaceBoundaries |> List.map (fun (p, _) -> { Position = p; Size = 0.01 }))

            (match actual.Root with
             | Leaf _ -> false
             | NonLeaf root when root.Length > 0 ->
                 match root[0] with
                 | Leaf l when l |> Set.count = tree.MaxLeafObjects + 1 ->
                     root
                     |> Array.skip 1
                     |> Array.forall (fun child ->
                         match child with
                         | Leaf l when l.IsEmpty -> true
                         | Leaf _ -> false
                         | NonLeaf _ -> false)
                 | Leaf _ -> false
                 | NonLeaf _ -> false
             | _ -> false)
            |@ $"before:{tree} actual:{actual}")
        |> Prop.forAll (treeGen |> Arb.fromGen)

    [<Property(EndSize = 100)>]
    let ``For tree with full Root node, inserting object works correctly if maxDepth is reached`` object =
        let maxSurfaceSize = 10.0

        let treeGen =
            Arb.generate<int>
            |> singleNodeTree maxSurfaceSize
            |> Gen.filter (fun tree -> tree.MaxDepth = 1)

        (fun tree ->
            // all objects are placed in the plane origin
            let actual =
                object
                |> SpatialTree.insert tree (fun _ ->
                    tree.SpaceBoundaries |> List.map (fun (p, _) -> { Position = p; Size = 0.01 }))

            (match actual.Root with
             | Leaf newRoot ->
                 match tree.Root with
                 | Leaf oldRoot when newRoot = Set.add object oldRoot -> true
                 | _ -> false
             | _ -> false)
            |@ $"before:{tree} actual:{actual}")

        |> Prop.forAll (treeGen |> Arb.fromGen)

    [<Property>]
    let internal ``inserting object outside of space boundaries causes exception``
        id
        (node: SpatialTreeNode<int>)
        (maxLeafObjects: int)
        (maxDepth: int)
        (spaceBoundaries: (float * float) NonEmptyList)
        =

        lazy
            (let tree =
                { Root = node
                  MaxLeafObjects = maxLeafObjects
                  MaxDepth = maxDepth
                  SpaceBoundaries = spaceBoundaries |> NonEmptyList.toList }

             id
             |> SpatialTree.insert tree (fun _ ->
                 { Size = 0.0; Position = 0.0 } |> List.replicate tree.SpaceBoundaries.Length))
        |> Prop.throws<ArgumentException, _>

    [<Fact>]
    let ``inserting object intersecting many subspaces works correctly for 1D simple`` () =

        let posMap =
            [ (1, { Position = 2.0; Size = 1.0 })
              (2, { Position = 7.0; Size = 1.0 })
              (3, { Position = 4.0; Size = 1.1 }) ]
            |> Map.ofList

        let tree = posMap |> singleNodeTreeFromPosMap [ (0, 10) ]

        let actual = SpatialTree.insert tree (fun id -> posMap[id] |> List.singleton) 3

        let expected =
            { tree with
                Root = [| [ 3; 1 ] |> Set.ofList |> Leaf; [ 3; 2 ] |> Set.ofList |> Leaf |] |> NonLeaf }

        Assert.Equal(expected, actual)

    [<Fact>]
    let ``inserting object works correctly for 1D simple`` () =
        let posMap =
            [ (1, { Position = 2.0; Size = 2.0 })
              (2, { Position = 7.0; Size = 1.0 })
              (3, { Position = 3.0; Size = 1.1 }) ]
            |> Map.ofList

        let tree = posMap |> singleNodeTreeFromPosMap [ (0, 10) ]

        let actual = SpatialTree.insert tree (fun id -> posMap[id] |> List.singleton) 3

        let expected =
            { tree with
                Root = [| [ 3; 1 ] |> Set.ofList |> Leaf; [ 2 ] |> Set.ofList |> Leaf |] |> NonLeaf }

        Assert.Equal(expected, actual)

    [<Fact>]
    let ``inserting object works correctly for 1D simple 2`` () =
        let posMap =
            [ (1, { Position = 12.0; Size = 2.0 })
              (2, { Position = 17.0; Size = 1.0 })
              (3, { Position = 18.0; Size = 1.1 }) ]
            |> Map.ofList

        let tree = posMap |> singleNodeTreeFromPosMap [ (10, 20) ]

        let actual = SpatialTree.insert tree (fun id -> posMap[id] |> List.singleton) 3

        let expected =
            { tree with
                Root = [| [ 1 ] |> Set.ofList |> Leaf; [ 3; 2 ] |> Set.ofList |> Leaf |] |> NonLeaf }

        Assert.Equal(expected, actual)

    [<Fact>]
    let ``inserting object works correctly for 1D complex`` () =
        let posMap =
            [ (1, { Position = 0; Size = 2.0 })
              (2, { Position = 6; Size = 1.0 })
              (3, { Position = 17.0; Size = 1.1 })
              (4, { Position = 1.0; Size = 1.1 }) ]
            |> Map.ofList

        let tree =
            { SpatialTree.init<int> 2 10 [ (0, 20) ] with
                Root = [| [ 1; 2 ] |> Set.ofList |> Leaf; [ 3 ] |> Set.ofList |> Leaf |] |> NonLeaf }

        let actual = SpatialTree.insert tree (fun id -> posMap[id] |> List.singleton) 4

        let expected =
            { tree with
                Root =
                    [| [| [ 4; 1 ] |> Set.ofList |> Leaf; [ 2 ] |> Set.ofList |> Leaf |] |> NonLeaf
                       [ 3 ] |> Set.ofList |> Leaf |]
                    |> NonLeaf }

        Assert.Equal(expected, actual)


    [<Fact>]
    let ``inserting object works correctly for 2D simple`` () =
        let posMap =
            [ (1, [ { Position = 2.0; Size = 2.0 }; { Position = 2.0; Size = 2.0 } ])
              (2, [ { Position = 2.0; Size = 1.0 }; { Position = 7.0; Size = 2.0 } ])
              (3, [ { Position = 7.0; Size = 1.0 }; { Position = 2.0; Size = 2.0 } ])
              (4, [ { Position = 7.0; Size = 1.0 }; { Position = 7.0; Size = 2.0 } ])
              (5, [ { Position = 1.0; Size = 1.1 }; { Position = 2.0; Size = 2.0 } ]) ]
            |> Map.ofList

        let tree = posMap |> singleNodeTreeFromPosMap [ (0, 10); (0, 10) ]

        let actual = SpatialTree.insert tree (fun id -> posMap[id]) 5

        let expected =
            { tree with
                Root =
                    [| [ 5; 1 ] |> Set.ofList |> Leaf
                       [ 2 ] |> Set.ofList |> Leaf
                       [ 3 ] |> Set.ofList |> Leaf
                       [ 4 ] |> Set.ofList |> Leaf |]
                    |> NonLeaf }

        Assert.Equal(expected, actual)
