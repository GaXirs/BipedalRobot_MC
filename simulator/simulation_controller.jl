## Import useful packages 
using LinearAlgebra
using StaticArrays
using StructArrays
using Plots
using RigidBodyDynamics
using RigidBodyDynamics.Contact
using StaticArrays
using Symbolics
using MeshCat, MeshCatMechanisms, Blink
using MechanismGeometries
using LaTeXStrings
using DelimitedFiles
using CSV
using DataFrames
using LightXML

## Include and import the ZMP based controller 
include(joinpath(@__DIR__, "RobotSimulator.jl"))
import .RobotSimulator

###########################################################
#                      Code parameters                    #
###########################################################

ANIMATE_RESULT = true;
write_torques = false;
data_from_CSV = false;

filename_read = joinpath(@__DIR__, "..", "data", "WP_validation_200Hz", "Outputs", "Torque.txt");
filename_save = joinpath(@__DIR__, "..", "data", "WalkingPattern", "Outputs", "Torque.txt");
#CSV_file = joinpath(@__DIR__, "..", "data", "WalkingPattern", "Raw", "walkingPattern_ref.csv");

# Asbtract trajectory 
CSV_file_one_sided = joinpath(@__DIR__, "..", "Dionysos_tests", "Biped_robot", "Dionysos_trajectory_one_sided.csv");
CSV_file_two_sided = joinpath(@__DIR__, "..", "Dionysos_tests", "Biped_robot", "Dionysos_trajectory_two_sided.csv");

# Pseudo concrete trajectory
CSV_pseudo_concrete_trajectory = joinpath(@__DIR__, "..", "Dionysos_tests", "Biped_robot", "Dionysos_trajectory_pseudo_concrete_trajectory.csv")

# Concrete trajectory
CSV_concrete_trajectory = joinpath(@__DIR__, "..", "Dionysos_tests", "Biped_robot", "Dionysos_trajectory_concrete_trajectory.csv")

###########################################################
#                         Simulation                      #
###########################################################

# Simulation parameters
robot_urdf = joinpath(@__DIR__, "..", "deps", "Robot_prismatic.urdf")
Δt = 1e-4       # Simulation step 

# Construct the robot in the simulation engine 
rs = RobotSimulator(;
    fileName = robot_urdf,
    symbolic = false,
    add_contact_points = true,
    add_gravity = true,
    add_flat_ground = true,
);

# Generate the visualiser
vis = set_visulalizer(; mechanism = rs.mechanism, fileName=robot_urdf)

# Initial configuration 
boom = [0, 0]
actuators = [0, 0, 0, 0]
foot = [0, 0]
set_nominal!(rs, vis, boom, actuators, foot)

if(data_from_CSV)
    Δt = 1e-4 # Do not change
    tend = 10.799
    
    folder = joinpath(@__DIR__, "..", "Dionysos_tests", "data", "Concrete")
    # Simulate the robot
    controller! = dynamixel_controller(rs, tend, Δt, CSV_concrete_trajectory, folder; freq=10.0, torque_model=torque_model, write_in_folder=true)
    ts, qs, vs = RigidBodyDynamics.simulate(rs.state, tend, controller!; Δt = Δt);
    println(qs[end][3:6])
    println(vs[end][3:6])
else
    tend = 2.0
    Δt_file = 0.005
    # Simulate the robot
    controller! = controller_torque_input_file(rs, tend, Δt_file, filename_read)
    ts, qs, vs = RigidBodyDynamics.simulate(rs.state, tend, controller!; Δt = Δt);
end

# Open the visulaiser and run the animation 
if ANIMATE_RESULT
    open(vis)
    sleep(10)

    animation = MeshCat.Animation(vis, ts, qs)
    setanimation!(vis, animation)
end