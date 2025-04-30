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
#                      Walking Pattern                    #
###########################################################

#ZMP
CSV_ZMP = joinpath(@__DIR__, "..", "WalkingPatterns", "ZMP.csv");

# Concrete trajectory
CSV_Julia_trajectory = joinpath(@__DIR__, "..", "WalkingPatterns", "Dionysos_trajectory.csv")

###########################################################
#                      Code parameters                    #
###########################################################

ANIMATE_RESULT = true;
data_from_WP = false;
write_output = false; #Note: increases the computational time like hell

freq = 10.0; # frequency of the CSV or the txt to read
tend = 10.0; # end time of the simulation

# For data_from_WP = false
filename_read = joinpath(@__DIR__, "..", "data", "Robot_200Hz", "Inputs", "Voltage.txt");

#For data_from_WP = true
WP_to_play = CSV_Julia_trajectory;
folder_save = joinpath(@__DIR__, "..", "data", "Dionysos", "Trajectory")

# URDF to used
# Simulation parameters
robot_urdf = joinpath(@__DIR__, "..", "deps", "Robot_prismatic.urdf")

###########################################################
#                         Simulation                      #
###########################################################

Δt = 1e-4 # Do not change

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

if(data_from_WP)
    # Simulate the robot
    controller! = dynamixel_controller(rs, tend, Δt, WP_to_play, folder_save; freq=freq, write_in_folder=write_output)
    @time begin
        ts, qs, vs = RigidBodyDynamics.simulate(rs.state, tend, controller!; Δt = Δt);
    end
else
    Δt_file = 1/freq
    # Simulate the robot
    folder = joinpath(@__DIR__, "..", "data", "simulation", "voltage-input")
    controller! = controller_voltage_input_file(rs, tend, Δt_file, filename_read, folder; write_in_folder=write_output)
    @time begin
        ts, qs, vs = RigidBodyDynamics.simulate(rs.state, tend, controller!; Δt = Δt);
    end
end

# Open the visulaiser and run the animation 
if ANIMATE_RESULT
    open(vis)
    sleep(10)

    animation = MeshCat.Animation(vis, ts, qs)
    setanimation!(vis, animation)
end