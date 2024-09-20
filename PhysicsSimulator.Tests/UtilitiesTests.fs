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

module internal SpatialTree =

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
        | Leaf l -> l = List.Empty
        | _ -> false

    [<Property>]
    let ``For empty tree getObjectBuckets returns empty seq`` () =

        (fun tree ->
            let actual = tree |> SpatialTree.getObjectBuckets
            let expected = Seq.empty
            actual .=. expected)
        |> Prop.forAll (Generators.SpatialTree.emptyTree 1000.0 |> Arb.fromGen)

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
                        extent |> List.map (fun (p, s) -> { Position = p; Size = s }))
            }

        (fun tree ->
            let actual = tree |> SpatialTree.getObjectBuckets
            let expected = object |> List.singleton |> Seq.singleton

            actual |> Seq.toList .=. (expected |> Seq.toList))
        |> Prop.forAll (treeGen |> Arb.fromGen)

    [<Property(EndSize = 10)>]
    let ``For tree with full Root node, inserting object causes node to split correctly if maxDepth is not reached``
        object
        =
        let maxSurfaceSize = 10.0

        let treeGen =
            Arb.generate<int>
            |> Generators.SpatialTree.singleNodeTree maxSurfaceSize
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
                 | Leaf l when l.Length = tree.MaxLeafObjects + 1 ->
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
            |> Generators.SpatialTree.singleNodeTree maxSurfaceSize
            |> Gen.filter (fun tree -> tree.MaxDepth = 1)

        (fun tree ->
            // all objects are placed in the plane origin
            let actual =
                object
                |> SpatialTree.insert tree (fun _ ->
                    tree.SpaceBoundaries |> List.map (fun (p, _) -> { Position = p; Size = 0.00 }))

            (match actual.Root with
             | Leaf newRoot ->
                 match tree.Root with
                 | Leaf oldRoot when newRoot = object :: oldRoot -> true
                 | _ -> false
             | _ -> false)
            |@ $"before:{tree} actual:{actual}")

        |> Prop.forAll (treeGen |> Arb.fromGen)

    [<Property(Arbitrary=[| typeof<NormalSpatialTree> |], EndSize = 1)>]
    let ``inserting object outside of space boundaries causes exception`` (tree: SpatialTree<int>)=
        lazy
            (1
             |> SpatialTree.insert tree (fun id ->
                 { Size = 0.0; Position = 0.0 } |> List.replicate tree.SpaceBoundaries.Length))
        |> Prop.throws<ArgumentException, _>

    [<Fact>]
    let ``inserting object intersecting many subspaces works correctly for 1D simple`` () =

        let posMap =
            [ (1, { Position = 2.0; Size = 1.0 })
              (2, { Position = 7.0; Size = 1.0 })
              (3, { Position = 4.0; Size = 1.1 }) ]
            |> Map.ofList

        let tree =
            { SpatialTree.init<int> 2 10 [ (0, 10) ] with
                Root = Leaf(posMap |> Map.keys |> Seq.take 2 |> Seq.toList) }

        let actual = SpatialTree.insert tree (fun id -> posMap[id] |> List.singleton) 3

        let expected =
            { tree with
                Root = [| [ 3; 1 ] |> Leaf; [ 3; 2 ] |> Leaf |] |> NonLeaf }

        Assert.Equal(expected, actual)

    [<Fact>]
    let ``inserting object works correctly for 1D simple`` () =
        let posMap =
            [ (1, { Position = 2.0; Size = 2.0 })
              (2, { Position = 7.0; Size = 1.0 })
              (3, { Position = 3.0; Size = 1.1 }) ]
            |> Map.ofList

        let tree =
            { SpatialTree.init<int> 2 10 [ (0, 10) ] with
                Root = Leaf(posMap |> Map.keys |> Seq.take 2 |> Seq.toList) }

        let actual = SpatialTree.insert tree (fun id -> posMap[id] |> List.singleton) 3

        let expected =
            { tree with
                Root = [| [ 3; 1 ] |> Leaf; [ 2 ] |> Leaf |] |> NonLeaf }

        Assert.Equal(expected, actual)

    [<Fact>]
    let ``inserting object works correctly for 1D simple 2`` () =
        let posMap =
            [ (1, { Position = 12.0; Size = 2.0 })
              (2, { Position = 17.0; Size = 1.0 })
              (3, { Position = 18.0; Size = 1.1 }) ]
            |> Map.ofList

        let tree =
            { SpatialTree.init<int> 2 10 [ (10, 20) ] with
                Root = Leaf(posMap |> Map.keys |> Seq.take 2 |> Seq.toList) }

        let actual = SpatialTree.insert tree (fun id -> posMap[id] |> List.singleton) 3

        let expected =
            { tree with
                Root = [| [ 1 ] |> Leaf; [ 3; 2 ] |> Leaf |] |> NonLeaf }

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
                Root = [| [ 1; 2 ] |> Leaf; [ 3 ] |> Leaf |] |> NonLeaf }

        let actual = SpatialTree.insert tree (fun id -> posMap[id] |> List.singleton) 4

        let expected =
            { tree with
                Root = [| [| [ 4; 1 ] |> Leaf; [ 2 ] |> Leaf |] |> NonLeaf; [ 3 ] |> Leaf |] |> NonLeaf }

        Assert.Equal(expected, actual)
