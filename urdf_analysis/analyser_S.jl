using RigidBodyDynamics
using MeshCat, MeshCatMechanisms

# Correct the URDF file path
urdf_path = joinpath(pwd(), "Robot_SD", "robot_SD.urdf")

# Load the URDF
robot = parse_urdf(Float64, urdf_path)

# Remove fixed joints and list joints
remove_fixed_tree_joints!(robot)
joints(robot)

# Create a visualizer
vis = MechanismVisualizer(robot, URDFVisuals(urdf_path))
state = MechanismState(robot)

# Extract joints
hip_right, hip_left, knee_right, knee_left = joints(robot)

# Set joint configurations
set_configuration!(state, hip_right, pi/2)
set_configuration!(state, knee_right, pi/2)
set_configuration!(state, hip_left, 0)
set_configuration!(state, knee_left, pi/2)

zero_velocity!(state)

# Ensure state updates properly
setdirty!(state)

q = configuration(state)
v = velocity(state)

# --- PRINT STATE ---
println("\n### FULL STATE ###")
println(state)

# Print configuration (joint positions)
println("\nJoint Positions: ", q)

# Print velocities
println("Joint Velocities: ", v)

# --- ACCESS SPECIFIC VALUES ---
joint_index = 1  # Change this to select another joint
velocity_index = 2  # Change this to select another velocity

println("\nChosen Joint Position [q[$joint_index]]: ", q[joint_index])
println("Chosen Joint Velocity [v[$velocity_index]]: ", v[velocity_index])

# --- MODIFY INDIVIDUAL VALUES ---
q[1] = 0.5  # Modify first joint position
set_configuration!(state, q)

v[2] = -0.2  # Modify second joint velocity
set_velocity!(state, v)

# Ensure state updates properly
setdirty!(state)

println("\nUpdated Joint Positions: ", configuration(state))
println("Updated Joint Velocities: ", velocity(state))

# Visualize the robot
open(vis)
