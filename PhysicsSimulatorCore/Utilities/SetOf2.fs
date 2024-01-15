namespace PhysicsSimulator.Utilities

type SetOf2<'t> =
    private
    | Value of List<'t> 

    member this.Get =
        match this with
        | Value v -> v

module SetOf2 =
    let create fst second = [ fst; second ] |> Value

    let ofList lst =
        match lst with
        | fst :: snd :: tail when tail.IsEmpty -> (fst, snd) ||> create
        | _ -> invalidArg "lst" "lst must be of size 2"

    let ofSet set = set |> Set.toList |> ofList

    let fst (set: SetOf2<'t>) = set.Get[0]
    let snd (set: SetOf2<'t>) = set.Get[1]

    let map f (set: SetOf2<'T>) = set.Get |> List.map f |> ofList
    let zip (set1: SetOf2<'T>) (set2: SetOf2<'V>) = set2.Get |> List.zip set1.Get |> ofList

    let zip3 (set1: SetOf2<'T>) (set2: SetOf2<'V>) (set3: SetOf2<'U>) =
        (set1.Get, set2.Get, set3.Get) |||> List.zip3 |> ofList




