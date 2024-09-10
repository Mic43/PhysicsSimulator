module PhisicsSimulator.Tests.Generators

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

    let properRangeGen =
        gen {
            let! l = Arb.generate<NormalFloat> |> Gen.map (_.Get)

            return! (l, Single.MaxValue |> float) |> genFloatInRange |> Gen.map (fun h -> (l, h))
        }

module SpatialTree =
    let emptyTree<'T> =
        (Arb.generate<PositiveInt> |> Gen.two, Common.properRangeGen |> Gen.nonEmptyListOf )
        ||> Gen.map2 (fun (maxLeafObjects, maxDepth) boundaries ->
            (maxLeafObjects.Get, maxDepth.Get, boundaries) |||> SpatialTree.init<'T>)

    let singleNodeTree<'T> (singleNodeGen: Gen<'T>) =
        gen {
            let! empty = emptyTree<'T>
            let! newRoot = singleNodeGen |> Gen.listOfLength empty.MaxLeafObjects |> Gen.map Leaf
            return { empty with Root = newRoot }
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

    let extent dimension =
        gen {
            let! b = Common.properRangeGen |> Gen.listOfLength dimension
            return! b |> extentInBoundaries
        }
