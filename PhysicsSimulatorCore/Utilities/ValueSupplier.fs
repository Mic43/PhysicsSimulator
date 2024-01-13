namespace PhysicsSimulator.Utilities

[<CustomEquality; NoComparison>]
type ValueSupplier<'T> when 'T: equality =  
    | Constant of 'T
    | Variable of (unit -> 'T)

    member this.GetValue() =
         match this with            
            | Constant vector3D -> vector3D
            | Variable unitFunc -> unitFunc ()
    override this.Equals(other) =
        match other with
        | :? ValueSupplier<'T> as other ->
            match (this, other) with         
            | Constant foo, Constant foo1 -> foo = foo1
            | _ -> false
        | _ -> false

