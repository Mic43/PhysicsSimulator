namespace PhysicsSimulator.Utilities

[<AutoOpen>]
module Constants =
    let epsilon = 0.00001

[<AutoOpen>]
module Utils =
    let rec subSetsOf2<'t when 't: comparison> (set: 't Set) =
        if (set.Count < 2) then
            Set.empty
        elif (set.Count = 2) then
            set |> Set.singleton
        else
            let maximumElement = set |> Set.toSeq |> Seq.head
            let subset = set |> Set.remove maximumElement

            subset
            |> subSetsOf2
            |> Set.union (subset |> Set.map (fun el -> [ el; maximumElement ] |> Set.ofList))

    [<TailCall>]
    let rec private subSetsOf2Ag<'t when 't: comparison> (processed: 't Set) rest accumulator =
        if rest |> Set.count = 0 then
            accumulator
        else
            let maximumElement = rest |> Set.toSeq |> Seq.head

            subSetsOf2Ag (processed |> Set.add maximumElement) (rest |> Set.remove maximumElement) accumulator
            |> Set.union (processed |> Set.map (fun el -> [ el; maximumElement ] |> Set.ofList))

    let subSetsOf2Tail set = subSetsOf2Ag Set.empty set Set.empty

    // Applies function n times to given object
    let applyN n f object =
        [ 1..n ] |> List.fold (fun obj _ -> obj |> f) object


    let equals (a: float) b = (b - a |> abs) <= Constants.epsilon
