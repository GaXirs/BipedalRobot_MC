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

## Include and import the ZMP based controller 
include(joinpath(@__DIR__, "..", "src", "ZMPBipedRobot.jl"))
import .ZMPBipedRobot

ZMProbot = ZMPBipedRobot

###########################################################
#                      Code parameters                    #
###########################################################
PLOT_RESULT = false;

ANIMATE_RESULT = true;

MODEL_2D = true;

write_torques = true;

data_from_CSV = true;

###########################################################
#                    Simulation parameters                #
###########################################################

if MODEL_2D
    ## Straight path for 2D Robot Model 
    t = vec(0:100)
    yPath = 1.18 .+ 0.0 .* t
    xPath = 0.01 * t
    θ_0 = 0
    robot_model = "ZMP_2DBipedRobot.urdf"
else
    ## Circle path for 3D Robot Model 
    t = vec(100:-1:75)
    xPath = -0 .- 1.18 * sin.(2 * pi / 100 .* t)
    yPath = 0 .+ 1.18 * cos.(2 * pi / 100 .* t)
    θ_0 = 0
    robot_model = "ZMP_3DBipedRobot.urdf"
end

lw = 2          # Plot line width 
dpi = 600
msw = 0.1
ms = 2

# Simulation parameters
Δt = 1e-3       # Simulation step 

# Position control parameters
Kp = 10000.0
Ki = 100.0
Kd = 100.0

# true : PD with dynamics compensation, false  : random torque 
ctrl = true

###########################################################
#                    ZMP based controller                 #
###########################################################

if(data_from_CSV)
    data = CSV.read(joinpath(@__DIR__, "walkingPattern_ref_short.csv"), DataFrame)
    # Extract data from the DataFrame
    tplot = data.time  # Extract the time column
    q1_l = data.q1_l   # Extract q1_l
    q1_r = data.q1_r   # Extract q1_r
    q2_l = data.q2_l   # Extract q2_l
    q2_r = data.q2_r   # Extract q2_r
    ZMPx = data.ZMPx   # Extract ZMPx
    ZMPy = data.ZMPy   # Extract ZMPy
    CoMx = data.CoMx   # Extract CoMx
    CoMy = data.CoMy   # Extract CoMy
    CoMz = data.CoMz   # Extract CoMz

    # Reconstruct qref, ZMP, and CoM
    qref = hcat(q1_l, q1_r, q2_l, q2_r)  # Reconstruct qref
    ZMP = hcat(ZMPx, ZMPy)               # Reconstruct ZMP
    CoM = hcat(CoMx, CoMy, CoMz)         # Reconstruct CoM
else
    # Construct the biped robot which store the geomtrical propreties and the path wanted 
    br = ZMProbot.BipedRobot(;
        readFile = true,
        URDFfileName = robot_model,
        paramFileName = "param.jl",
    )
    br.xPath = xPath;
    br.yPath = yPath;
    br.initial_position = [xPath[1], yPath[1], θ_0]

    # Construct the Preview Controller
    pc = ZMProbot.PreviewController(; br = br, check = PLOT_RESULT)

    # Run the Foot Planer Algorithm and get the foot position 
    fp = ZMProbot.FootPlanner(; br = br, check = PLOT_RESULT)

    # Get the ZMP reference trajectory 
    zt = ZMProbot.ZMPTrajectory(; br = br, fp = fp, check = PLOT_RESULT)

    # Convert the ZMP reference trajectory into CoM trajectory
    ct = ZMProbot.CoMTrajectory(; br = br, pc = pc, zt = zt, check = PLOT_RESULT)

    # Get the Swing Foot trajectory 
    sf = ZMProbot.SwingFootTrajectory(; br = br, fp = fp, zt = zt, check = PLOT_RESULT)

    # Get the joint trajectory from the all path 
    ik = ZMProbot.InverseKinematics(; br = br, fp = fp, ct = ct, sf = sf, check = PLOT_RESULT)

    # Store into more convienant variables 
    qr = ik.q_r;
    ql = ik.q_l;
    qref = [ql[:, 1] qr[:, 1] ql[:, 2] qr[:, 2]]

    CoM = reduce(hcat, ct.CoM)
    ZMP = reduce(hcat, zt.ZMP)
    tplot = reduce(vcat, zt.timeVec)
end

###########################################################
#                  Simulation environement                #
###########################################################
tend = tplot[end]       # Simulation time 
# Construct the robot in the simulation engine 
rs = ZMProbot.RobotSimulator(;
    fileName = robot_model,
    symbolic = false,
    add_contact_points = true,
    add_gravity = true,
    add_flat_ground = true,
);
# Generate the visualiser 
vis = ZMProbot.set_visulalizer(; mechanism = rs.mechanism)

# Intiial configuration 
boom = [0, 0]
actuators = [0, 0, 0, 0]
foot = [0, 0]
ZMProbot.set_nominal!(rs, vis, boom, actuators, foot)

# Simulate the robot
if(write_torques)
    open("torques.txt", "w") do file
        # The file is now open in write mode, and all its contents are deleted.
        # Do nothing if you don't want to write anything.
    end
end
controller! = ZMProbot.trajectory_controller!(rs, tplot, qref, Δt, Kp, Ki, Kd, ctrl, write_torques)
ts, qs, vs = RigidBodyDynamics.simulate(rs.state, tend, controller!; Δt = Δt);

# Open the visulaiser and run the animation 
if ANIMATE_RESULT
    open(vis)
    sleep(10)
    MeshCatMechanisms.animate(vis, ts, qs)
end