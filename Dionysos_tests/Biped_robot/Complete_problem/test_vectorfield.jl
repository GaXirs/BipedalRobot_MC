include("robot_problem.jl")
using .RobotProblem
using StaticArrays
using Printf
using RigidBodyDynamics
using MeshCat, MeshCatMechanisms, Blink
using MechanismGeometries


# Define a sample state x (positions and velocities)
# Assuming the robot starts in a neutral position
x_test = SVector{8}(
    0, 0.0, 
    0, 0.0,
    0.0, 0.0, 
    0.0, 0.0 
)

# Define a sample input u (voltages to motors)
u_test = SVector{4}(0.0, 1.0, 0.0, 0.0)  # Example voltage inputs

# Call the vector field function
x_next,full_state,ts,qs,vs = RobotProblem.vectorFieldBipedRobot(x_test, u_test)
q, dq = RobotProblem.fill_state!(x_next)
filled_state = [q..., dq...]

println("Formatted full_state  : [", join(round.(full_state, digits=10), ", "), "]")
println("Formatted filled_state: [", join(round.(filled_state, digits=3), ", "), "]")
println("Formatted difference  : [", join(round.(filled_state .- full_state, digits = 8), ", "), "]")
println("Formatted percentage  : [", join(round.((filled_state .- full_state)./ full_state, digits = 3), ", "), "]")

# Print the result
#println("Next state: ", x_next)

# Load the URDF from the current folder
"""
urdfpath() = joinpath(@__DIR__, "..", "deps",  "ZMP_2DBipedRobot_nodamping.urdf")
mechanism = RigidBodyDynamics.parse_urdf(urdfpath())
state = MechanismState(mechanism)

vis = MechanismVisualizer(mechanism, URDFVisuals(urdfpath()))
q = full_state[1:8]
set_configuration!(state, q)
zero_velocity!(state)

set_configuration!(vis, RigidBodyDynamics.configuration(state))

robot_bodies = RigidBodyDynamics.bodies(mechanism)
for body in robot_bodies
    frame = RigidBodyDynamics.default_frame(body)
    setelement!(vis, frame)
end
"""
# Visualize the robot

open(vis)
sleep(2)
MeshCatMechanisms.animate(vis, ts, qs)