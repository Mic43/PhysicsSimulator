module PhisicsSimulator.Tests.Generators

open FSharpPlus.Data
open FsCheck
open PhysicsSimulator.Utilities

module SpatialTree =
    let emptyTree<'T> =
        (Arb.generate<PositiveInt> |> Gen.two,
         Arb.Default.NormalFloat().Generator
         |> Gen.map (_.Get)
         |> Gen.two
         |> Gen.nonEmptyListOf)
        ||> Gen.map2 (fun (maxLeafObjects, maxDepth) boundaries ->
            (maxLeafObjects.Get, maxDepth.Get, boundaries) |||> SpatialTree.init)
