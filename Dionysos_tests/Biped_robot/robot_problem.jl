module RobotProblem

using MathematicalSystems
using LinearAlgebra, StaticArrays
using RigidBodyDynamics

# include the tools for the simulator from src
include(joinpath(@__DIR__, "src", "RS_tools.jl"))
import .RS_tools


robot_urdf = joinpath(@__DIR__, "deps/ZMP_2DBipedRobot_nodamping.urdf")
rs = RS_tools.RobotSimulator(;
    fileName = robot_urdf,
    symbolic = false,
    add_contact_points = true,
    add_gravity = true,
    add_flat_ground = true,)

mechanism = rs.mechanism
state = MechanismState(mechanism)
n_pos = num_positions(state)
n_vel = num_velocities(state)
Δt_simu     = 1e-4       # Simulation step 
Δt_dionysos = 3     # Dinoysos time discretisation, nominal 50Hz (control freq of the material robot)

println("n_pos: ", n_pos)
println("n_vel: ", n_vel)


## MOTOR Parameters ##
HGR = 353.5           # Hip gear-ratio
KGR = 212.6           # Knee gear-ratio
ktp  = 0.395/HGR      # Torque constant with respect to the voltage [Nm/V] 
Kvp  = 1.589/(HGR*HGR)      # Viscous friction constant [Nm*s/rad] (linked to motor speed)
τc_u  = 0.065/HGR           # Dry friction torque [Nm]
τ_m = [0.0,0.0,0.0,0.0]
# Discrete time using Rigibodydynamics simulator -> returns (X[i], U[i]) -> X[i+1]
function voltage_controller!(
    u:: SVector,
)
    ddl = 2
    function controller!(τ, t, state)
        τ .= 0
        current_̇q = velocity(state)[(end - 3 - ddl):(end - ddl)]
        ω = current_̇q .* [HGR, HGR, KGR, KGR]

        τ_0 = u .* [HGR, HGR, KGR, KGR] .* ktp  .- ω .* [HGR, HGR, KGR, KGR] .* Kvp
        τ_m .= τ_0 .- sign.(ω) .* [HGR, HGR, KGR, KGR] .* τc_u
        τ[(end - 3 - ddl):(end - ddl)] .= τ_m
    end
end

function DXL_controller!(
    q_ref::SVector
)
    ddl=2
    Kp = 900.0 / 128.0
    PWM_goal = 885.0
    Nominal_voltage = 12.0

    current_q = [0.0,0.0,0.0,0.0]
    u = [0.0,0.0,0.0,0.0]
    ω = [0.0,0.0,0.0,0.0]
    τ_m = [0.0,0.0,0.0,0.0]

    function controller!(τ, t, state)
        current_q .= configuration(state)[(end - 3 - ddl):(end - ddl)]
        current_̇q = velocity(state)[(end - 3 - ddl):(end - ddl)]

        PWM = (q_ref .- current_q) .* (4095.0/(2π)* Kp) # Only true because profile acceleration and profile velocity are null
        PWM_sat = clamp.(PWM, -PWM_goal, PWM_goal)# Apply_saturation

        u .= PWM_sat .* (Nominal_voltage / 885.0)
        ω .= current_̇q .* [HGR, HGR, KGR, KGR]

        τ_0 = u .* [HGR, HGR, KGR, KGR] .* ktp  .- ω .* [HGR, HGR, KGR, KGR] .* Kvp
        τ_m .= τ_0 .- sign.(ω) .* [HGR, HGR, KGR, KGR] .* τc_u

        τ[(end - 3 - ddl):(end - ddl)] .= τ_m
    end
end

## Robots Parameters ##
Lthigh = 0.20125
Lleg = 0.172
Hip_offset = 0.04025
Foot_height = 0.009
Init_offset = -0.0006559432
function fill_state!(x)
    # Create q
    q = vcat(zeros(2), x[1:4], zeros(2))
    q̇ = vcat(zeros(2), x[5:8], zeros(2))
    
    # Compute the heights of the two legs (double pendulums)
    zl = Lthigh * cos(q[3]) + Lleg * cos(q[5] + q[3])
    zr = Lthigh * cos(q[4]) + Lleg * cos(q[6] + q[4])

    # FILL THE POSITIONS
    
    # Write the maximum height to q[2]
    # (adding the distance from the hip joint to hip body and the height of the foot)
    # The most extended leg is in contact with the ground

    q[2] = max(zl, zr) - Lthigh - Lleg + Init_offset
    
    # Set additional constraints
    # The x position is set to 0
    # The feet are kept // to the ground 
    
    q[7] = -(q[3] + q[5])
    q[8] = -(q[4] + q[6])

    # FILL THE SPEEDS
    # identify the contact leg
    i1 = 0
    i2 = 0
    if (zl > zr)
        i1,i2 = 3,5
    else 
        i1,i2 = 4,6
    end
    # speed equations of the double pendulum
    x = Lthigh * sin(q[i1]) + Lleg * sin(q[i2] + q[i1])
    ẋ = Lthigh * q̇[i1] * cos(q[i1]) + Lleg * (q̇[i1] + q̇[i2]) * cos(q[i1] + q[i2])
    ż = -(Lthigh * q̇[i1] * sin(q[i1]) + Lleg * (q̇[i1] + q̇[i2]) * sin(q[i1] + q[i2]))
    q[1] = x
    q̇[1] = ẋ
    q̇[2] = ż

    # adjust the angular speed of the feet to remain mostly horizontal
    q̇[7] = -(q̇[3] + q̇[5])
    q̇[8] = -(q̇[4] + q̇[6])
    return q, q̇
end

function vectorFieldBipedRobot(x, u)
    # Variables: [x z LH RH LK RK LA RA]
    # NB: to move the knee forward, a negative angle is needed!
    q, q̇ = fill_state!(x)

    set_configuration!(state, q)
    set_velocity!(state, q̇)

    controller! = DXL_controller!(u)
    ts, qs, vs  = RigidBodyDynamics.simulate(state, Δt_dionysos, controller!; Δt = Δt_simu);
    
    x_next = SVector{length(x)}(qs[end][3:6]..., vs[end][3:6]...)
    full_state = SVector{2 * length(qs[end])}(qs[end]..., vs[end]...)
    
    #println("Output of simulation")
    #println(full_state)

    return x_next, full_state
end

end