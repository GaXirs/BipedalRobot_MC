using RigidBodyDynamics
using MeshCat, MeshCatMechanisms, Blink
using MechanismGeometries

# Load the URDF from the current folder
#urdfpath()= joinpath(pwd(), "Robot_SD_fixed.urdf")
urdfpath() = joinpath(pwd(),"Robot_SD", "robot_SD.urdf")
mechanism = RigidBodyDynamics.parse_urdf(urdfpath())
state = MechanismState(mechanism)

vis = MechanismVisualizer(mechanism, URDFVisuals(urdfpath()))
q = [0,0,0,0,0,0,0,1]
set_configuration!(state, q)
zero_velocity!(state)

set_configuration!(vis, RigidBodyDynamics.configuration(state))

robot_bodies = RigidBodyDynamics.bodies(mechanism)
for body in robot_bodies
    frame = RigidBodyDynamics.default_frame(body)
    setelement!(vis, frame)
end

# Visualize the robot
open(vis)
