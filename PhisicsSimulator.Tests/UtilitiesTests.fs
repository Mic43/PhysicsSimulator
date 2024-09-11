module UtilitiesTests

open System
open FSharpPlus.Data
open FsCheck
open FsCheck.Xunit
open PhysicsSimulator.Utilities

let (.=.) left right =
    left = right |@ sprintf "%A = %A" left right

module SpatialTree =

    open PhisicsSimulator.Tests

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
                        extent |> List.map (fun (p, s) -> {| Position = p; Size = s |}))
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
                    tree.SpaceBoundaries |> List.map (fun (p, _) -> {| Position = p; Size = 0.00 |}))

            match actual.Root with
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
        |> Prop.forAll (treeGen |> Arb.fromGen)
