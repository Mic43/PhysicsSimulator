module UtilitiesTests

open System
open FSharpPlus.Data
open FsCheck
open FsCheck.Xunit
open PhysicsSimulator.Utilities


let (.=.) left right =
    left = right |@ sprintf "%A = %A" left right

module SpatialTree =

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
        |> Prop.forAll (PhisicsSimulator.Tests.Generators.SpatialTree.emptyTree |> Arb.fromGen)

    [<Property>]
    let ``For single object tree getObjectBuckets returns singleton `` (object: int) =
     
        let extentGen dimension =
            gen {
                let! b = PhisicsSimulator.Tests.Generators.Common.properRangeGen |> Gen.listOfLength dimension
                return! b |> PhisicsSimulator.Tests.Generators.SpatialTree.extendInBoundaries
            }

        let treeGen =
            gen {
                let! emptyTree = PhisicsSimulator.Tests.Generators.SpatialTree.emptyTree<int>
                let! extent = emptyTree.SpaceBoundaries.Length |> extentGen
                
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
