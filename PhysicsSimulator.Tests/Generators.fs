module PhysicsSimulator.Tests.Generators

open System
open FSharpPlus.Data
open FsCheck
open PhysicsSimulator.Utilities

module Common =
    let genFloatInRange (min, max) =
        gen {
            let! n = Arb.generate<PositiveInt>
            let f = float n.Get / float Int32.MaxValue
            return min + (f * (max - min))
        }

    let properRangeGen max =
        gen {
            let! l = Arb.generate<NormalFloat> |> Gen.map (_.Get)

            return!
                (l, (float Int32.MaxValue * max))
                |> genFloatInRange
                |> Gen.map (fun h -> (l, h))
        }

module internal SpatialTree =
    let leaf objects : SpatialTreeNode<'T> = { Kind = Leaf objects; Parent = None }

    let nonLeaf (children: SpatialTreeNode<'T> array) : SpatialTreeNode<'T> =
        { Kind = NonLeaf children; Parent = None }

    let rec kindsEqual (a: SpatialTreeNode<'T>) (b: SpatialTreeNode<'T>) =
        match a.Kind, b.Kind with
        | Leaf la, Leaf lb -> la = lb
        | NonLeaf ca, NonLeaf cb when ca.Length = cb.Length -> Array.forall2 kindsEqual ca cb
        | _ -> false

    let treesEqual (expected: SpatialTree<'T>) (actual: SpatialTree<'T>) =
        expected.MaxLeafObjects = actual.MaxLeafObjects
        && expected.MaxDepth = actual.MaxDepth
        && expected.SpaceBoundaries = actual.SpaceBoundaries
        && kindsEqual expected.Root actual.Root

    let emptyTree<'T when 'T: comparison> maxSize =
        (Arb.generate<PositiveInt> |> Gen.two, Common.properRangeGen maxSize |> Gen.nonEmptyListOf)
        ||> Gen.map2 (fun (maxLeafObjects, maxDepth) boundaries ->
            (maxLeafObjects.Get, maxDepth.Get, boundaries) |||> SpatialTree.init<'T>)

    let distinctSetOfLength n (itemGen: Gen<'T>) =
        let rec build remaining acc =
            gen {
                if remaining <= 0 then
                    return acc |> Set.ofList
                else
                    let! item = itemGen |> Gen.filter (fun x -> acc |> List.contains x |> not)
                    return! build (remaining - 1) (item :: acc)
            }

        build n []

    let singleNodeTree<'T when 'T: comparison> maxSize (singleNodeGen: Gen<'T>) =
        gen {
            let! empty = emptyTree<'T> maxSize
            let! objects = distinctSetOfLength empty.MaxLeafObjects singleNodeGen
            return { empty with Root = leaf objects }
        }

    let extentInBoundary (boundary: float * float) =
        if boundary |> fst > (boundary |> snd) then
            invalidArg "boundary" "invalid boundary"

        gen {
            let! pos = boundary |> Common.genFloatInRange
            let! pos2 = (pos, boundary |> snd) |> Common.genFloatInRange

            return (pos, pos2 - pos)
        }

    let extentInBoundaries (boundary: (float * float) list) =
        boundary |> List.map extentInBoundary |> Gen.sequence

    let extent maxSize dimension =
        gen {
            let! b = Common.properRangeGen maxSize |> Gen.listOfLength dimension
            return! b |> extentInBoundaries
        }
    type NormalSpatialTree =
        static member Double() =
            Arb.generate<SpatialTree<int>>
            |> Gen.filter (_.SpaceBoundaries.IsEmpty >> not)
            |> Arb.fromGen
