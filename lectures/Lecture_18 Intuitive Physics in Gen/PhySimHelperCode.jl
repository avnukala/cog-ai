import Pkg
using PyCall
using Parameters
Pkg.add("StaticArrays")
using StaticArrays
Pkg.add("DocStringExtensions")
using DocStringExtensions

# const pb = PyNULL()

# function __init__()
#     copy!(pb, pyimport("pybullet"))
# end


""" Parameters for a physics engine """
abstract type PhySim end

""" The result of simulation """
abstract type PhyState{T<:PhySim} end

""" A component in simulation """
abstract type Element{T<:PhySim} end

""" The state of an element """
abstract type ElemState{T<:Element} end

""" The latents describing the dynamics of an element """
abstract type ElemLatents{T<:Element} end

"""
     step(sim::PhySim, st::PhyState)::SimState

Performs a stateless evolution of the simulation state.
"""
function step(sim::PhySim, st::PhyState)
    sync!(sim, st)
    new_st = forward_step(sim, st)
end


"""
    sync!(sim::PhySim, st::PhyState)::Nothing

Synchronizes the context within `sim` using `st`.
"""
function sync! end

"""
    forward_step(sim::PhySim)::PhyState

Resolves physical interactions and obtains the next state representation.
"""
function forward_step end


@with_kw struct BulletSim <: PhySim
    "Client id for pybullet"
    client::Int64
    "Amount of time between `forward_steps` (default=16.7ms)"
    step_dur::Float64 = 1 / 60
    "Timestep duration of bullet engine (default: 4.2ms)"
    pb_timestep::Float64 = 1 / 240
end


""" An element for `BulletSim` """
abstract type BulletElement <: Element{BulletSim} end
abstract type BulletElemState{T<:BulletElement} <: ElemState{T} end
abstract type BulletElemLatents{T<:BulletElement} <: ElemLatents{T} end

function get_state end

function set_state! end

function get_latents end

function set_latents! end

"""
State for `BulletSim`

$(TYPEDEF)

---

$(TYPEDFIELDS)
"""
struct BulletState <: PhyState{BulletSim}
    elements::AbstractVector{BulletElement}
    latents::AbstractVector{BulletElemLatents}
    kinematics::AbstractVector{BulletElemState}
end

function BulletState(sim::BulletSim,
                     elements::AbstractVector{T}) where {T<:BulletElement}
    latents = map(x -> get_latents(x, sim), elements)
    kinematics = map(x -> get_state(x, sim), elements)
    BulletState(elements, latents, kinematics)
end

function sync!(sim::BulletSim, world_state::BulletState)
    for (elem, ls, est) in zip(world_state.elements,
                               world_state.latents,
                               world_state.kinematics)
        set_state!(elem, sim, est)
        set_latents!(elem, sim, ls)
    end
    return nothing
end

function forward_step(sim::BulletSim, st::BulletState)
    # progress by `st.step_dur`
    dt::Float64 = 0.0
    while dt <= sim.step_dur
        @pycall pb.stepSimulation(;
                                  physicsClientId = sim.client
                                  )::PyObject
        dt += sim.pb_timestep
    end
    # extract resulting state
    ne = length(st.elements)
    elements = Vector{BulletElement}(undef, ne)
    latents = Vector{BulletElemLatents}(undef, ne)
    kinematics = Vector{BulletElemState}(undef, ne)
    @inbounds for i = 1:ne
        elements[i] = st.elements[i]
        latents[i] = st.latents[i] # REVIEW: this vs `get_latents`
        kinematics[i] = get_state(elements[i], sim)
    end
    BulletState(elements, latents, kinematics)
end

include("rigid_body.jl")

