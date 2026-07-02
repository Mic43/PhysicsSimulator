namespace PhysicsSimulator.Utilities

open System
open FSharpPlus


[<RequireQualifiedAccess>]
[<CustomEquality; NoComparison>]
type Pair<'t when 't: equality> =
    private
    | Value of list<'t>

    member this.Get =
        match this with
        | Value v -> v

    override this.Equals other =
        match other with
        | :? Pair<'t> as otherPair ->
            otherPair.Get[0] = this.Get[0] && otherPair.Get[1] = this.Get[1]
        | _ -> false

    override this.GetHashCode() =
        HashCode.Combine(this.Get[0], this.Get[1])

module internal Pair =
    let create fst second = [ fst; second ] |> Pair.Value
    let ofPair pair = pair ||> create

    let ofList lst =
        match lst with
        | fst :: snd :: tail when tail.IsEmpty -> (fst, snd) ||> create
        | _ -> invalidArg "lst" "lst must be of size 2"

    let ofSet set = set |> Set.toList |> ofList
    let toList (pair: Pair<'t>) = pair.Get
    let toSet (pair: Pair<'t>) = pair |> toList |> Set.ofList
    let fst (pair: Pair<'t>) = pair.Get[0]
    let snd (pair: Pair<'t>) = pair.Get[1]
    let toTuple (pair: Pair<'t>) = (pair |> fst, pair |> snd)

    /// Canonical key for deduplicating unordered pairs (e.g. broad-phase candidates).
    let unorderedKey<'t when 't: comparison> (pair: Pair<'t>) =
        let a, b = toTuple pair

        if compare a b <= 0 then
            struct (a, b)
        else
            struct (b, a)

    let map f (pair: Pair<'T>) = pair.Get |> List.map f |> ofList

    let join (pairs: Pair<Pair<'T>>) = pairs |> toList |> List.collect toList
    let fold folder (state: 'State) pair = pair |> toList |> List.fold folder state
    let zip (pair1: Pair<'T>) (pair2: Pair<'V>) = pair2.Get |> List.zip pair1.Get |> ofList

    let zip3 (pair1: Pair<'T>) (pair2: Pair<'V>) (pair3: Pair<'U>) =
        (pair1.Get, pair2.Get, pair3.Get) |||> List.zip3 |> ofList

    let unzip (pair: Pair<'T * 'V>) =
        let (a, b) = pair.Get |> List.unzip
        (a |> ofList, b |> ofList)

    let flip pair = pair |> toList |> List.rev |> ofList
    let max pair = pair |> toList |> List.max
