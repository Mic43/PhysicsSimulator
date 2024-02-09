namespace PhysicsSimulator.Utilities


[<RequireQualifiedAccess>]
type SetOf2<'t> =
    private
    | Value of List<'t>

    member this.Get =
        match this with
        | Value v -> v

module SetOf2 =
    let create fst second = [ fst; second ] |> SetOf2.Value
    let ofPair pair = pair ||> create

    let ofList lst =
        match lst with
        | fst :: snd :: tail when tail.IsEmpty -> (fst, snd) ||> create
        | _ -> invalidArg "lst" "lst must be of size 2"

    let ofSet set = set |> Set.toList |> ofList
    let toList (set: SetOf2<'t>) = set.Get
    let toSet (set: SetOf2<'t>) = set |> toList |> Set.ofList
    let fst (set: SetOf2<'t>) = set.Get[0]
    let snd (set: SetOf2<'t>) = set.Get[1]

    let map f (set: SetOf2<'T>) = set.Get |> List.map f |> ofList
    let zip (set1: SetOf2<'T>) (set2: SetOf2<'V>) = set2.Get |> List.zip set1.Get |> ofList

    let zip3 (set1: SetOf2<'T>) (set2: SetOf2<'V>) (set3: SetOf2<'U>) =
        (set1.Get, set2.Get, set3.Get) |||> List.zip3 |> ofList

    let flip set = set |> toList |> List.rev |> ofList
    let max set = set |> toList |> List.max