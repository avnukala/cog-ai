export RigidBody,
    RigidBodyState,
    RigidBodyLatents

"""
A rigid body in `BulletSim`

$(TYPEDEF)

Consists of a single base collision object.

---

$(TYPEDFIELDS)
"""
struct RigidBody <: BulletElement
    "The `bodyUniqueId` of the base object"
    object_id::Int64
end

"""
State for `RigidBody`

$(TYPEDEF)

---

$(TYPEDFIELDS)
"""
struct RigidBodyState <: BulletElemState{RigidBody}
    "XYZ position"
    position::SVector{3, Float64}
    "Quaternion xyzw"
    orientation::SVector{4, Float64}
    "Linear velocity XYZ"
    linear_vel::SVector{3, Float64}
    "Angular velocity wX wY wZ"
    angular_vel::SVector{3, Float64}
    "Axis-aligned Bounding box"
    aabb::SVector{2, SVector{3, Float64}}
end

function RigidBodyState(e::RigidBody, sim::BulletSim)
    get_state(e, sim)
end

function get_state(e::RigidBody, sim::BulletSim)
    (pos, orn) =
        @pycall pb.getBasePositionAndOrientation(;
                                                 # docstring is wrong
                                                 # about keyword name
                                                 bodyUniqueId =  e.object_id,
                                                 physicsClientId = sim.client,
                                                 )::Tuple{PyVector, PyVector}
    (lin_vel, ang_vel) =
        @pycall pb.getBaseVelocity(;
                                   bodyUniqueId =  e.object_id,
                                   physicsClientId = sim.client
                                   )::Tuple{PyVector, PyVector}
    aabb =
        @pycall pb.getAABB(;
            bodyUniqueId = e.object_id,
            linkIndex = -1, # base (assumption for `RigidBody`)
            physicsClientId = sim.client
        )::PyObject
    RigidBodyState(pos, orn, lin_vel, ang_vel, [collect(aabb[i]) for i in 1:2])
end

function set_state!(e::RigidBody, sim::BulletSim, st::RigidBodyState)
    @pycall pb.resetBasePositionAndOrientation(;
                                               bodyUniqueId =  e.object_id,
                                               # linkIndex = -1,
                                               posObj =  st.position,
                                               ornObj = st.orientation,
                                               physicsClientId = sim.client
                                               )::PyObject
    @pycall pb.resetBaseVelocity(;
                                 # the different name is annoying
                                 objectUniqueId =  e.object_id,
                                 # linkIndex = -1,
                                 linearVelocity = st.linear_vel,
                                 angularVelocity = st.angular_vel,
                                 physicsClientId = sim.client
                                 )::PyObject

    return nothing
end

"""
Latents for `RigidBody`

$(TYPEDEF)

Any collection of property values can be declared in `data`.
Most commonly these will properties such as "mass" or "lateralFriction"
and any undeclared properties will use default values (see pybullet)

A non-exhaustive list of properties
- mass
- lateralFriction
- localInertiaDiagonal
- localInertialPos
- localInertialOrn
- restitution
- rollingFriction
- spinningFriction
- contactDamping
- contactStiffness

---

$(TYPEDFIELDS)
"""
struct RigidBodyLatents <: BulletElemLatents{RigidBody}
    data::NamedTuple
end

function get_latents(e::RigidBody, sim::BulletSim)
    # REVIEW: pybullet docs says `getDynamicsInfo` is incomplete / weird
    ls =
        @pycall pb.getDynamicsInfo(;
                                   bodyUniqueId = e.object_id,
                                   linkIndex = -1, # base (assumption for `RigidBody`)
                                   physicsClientId = sim.client
                                   )::PyObject
    RigidBodyLatents((mass = ls[1],
                      lateralFriction = ls[2],
                      # localInertiaDiagonal = ls[3],
                      # localInertialPos = ls[4],
                      # localInertialOrn = ls[5],
                      restitution = ls[6],
                      rollingFriction = ls[7],
                      spinningFriction = ls[8],
                      contactDamping = ls[9],
                      contactStiffness = ls[10],
                      collisionMargin = ls[12]))
end

function set_latents!(e::RigidBody, sim::BulletSim, ls::RigidBodyLatents)
    
    @pycall pb.changeDynamics(;
        bodyUniqueId = e.object_id,
        linkIndex = -1, # base (assumption for `RigidBody`)
        ls.data..., # REVIEW: more direct?
        physicsClientId = sim.client
    )::PyObject
    return nothing
end
