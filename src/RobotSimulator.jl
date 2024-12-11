"""

A Robot Simulator allows to define the simulation envrionnment.

A `RobotSimulator` has 5 differents outputs :
    1.  `mechanism` which is a data structure which contains the state of the robot and more 
    2.  `state` which is the actual state of the robot 
    3.  `CoM` which is the an array with the simulated CoM trajectory 
    4.  `torques` which is the an array with the simulated joints torques trajectory 
    5.  `ZMP` which is the an array with the simulated ZMP trajectory 
    
"""

struct RobotSimulator
    mechanism::Mechanism
    state::MechanismState
    CoM::Array
    torques::Array
    ZMP::Array
end

"""

Constructor 
"""
function RobotSimulator(;
    fileName::String = "ZMP_2DBipedRobot.urdf",
    symbolic::Bool = false,
    use_urdf::Bool = !symbolic,
    T::Type = symbolic ? Num : Float64,
    add_contact_points = true,
    add_flat_ground = true,
    add_gravity = true,
    contactmodel = default_contact_model(),
)
    robot = getMechanism(;
        fileName,
        symbolic,
        use_urdf,
        T,
        add_contact_points,
        add_flat_ground,
        add_gravity,
        contactmodel,
    )
    state = MechanismState(robot)
    return RobotSimulator(robot, state, Array[], Array[], Array[])
end

"""

Define the contact model and their parameters 
"""
function default_contact_model()
    return SoftContactModel(
        hunt_crossley_hertz(; k = 500e2),
        ViscoelasticCoulombModel(0.8, 20e3, 100.0),
    )
end

"""

Return the URDF  `mechanism` of the robot. 

Describe the simulation environement as well. 
"""
function getMechanism(;
    fileName::String = "ZMP_bipedalRobot.urdf",
    symbolic::Bool = false,
    use_urdf::Bool = !symbolic,
    T::Type = symbolic ? Num : Float64,
    add_contact_points = true,
    add_flat_ground = true,
    add_gravity = true,
    contactmodel = default_contact_model(),
)
    urdfpath() = joinpath(packagepath(), fileName)

    # Define the robot mechanism
    if use_urdf
        mechanism = RigidBodyDynamics.parse_urdf(urdfpath())
        visual = URDFVisuals(urdfpath())
        if (add_gravity)
            mechanism.gravitational_acceleration =
                FreeVector3D(root_frame(mechanism), 0, 0, -9.81)
        else
            mechanism.gravitational_acceleration =
                FreeVector3D(root_frame(mechanism), 0, 0, 0)
        end
    else
        error("Not implemented yet ! Please use URDF file !")
    end
    # Configuration of the contact points
    if !symbolic && add_contact_points && contactmodel !== nothing
        e = root(visual.xdoc)
        s = attribute(e["link"][end]["visual"][1]["geometry"][1]["box"]..., "size")

        numbers = split(s, " ")
        left_foot_length = parse(Float64, numbers[1])
        left_foot_width = parse(Float64, numbers[2])
        left_foot_height = parse(Float64, numbers[3])

        for side in (:left, :right)
            foot_link = findbody(mechanism, "$(first(string(side)))_foot_link")
            frame = default_frame(foot_link)
            cp0 = Point3D(frame, zero(T), zero(T), -left_foot_height)
            add_contact_point!(foot_link, ContactPoint(cp0, contactmodel))
            for div in 2:2
                for sign in [-1, 1]
                    cp = Point3D(
                        frame,
                        sign * left_foot_length / div,
                        sign * left_foot_width / div,
                        -left_foot_height,
                    )
                    add_contact_point!(foot_link, ContactPoint(cp, contactmodel))
                    cp = Point3D(
                        frame,
                        -sign * left_foot_length / div,
                        sign * left_foot_width / div,
                        -left_foot_height,
                    )
                    add_contact_point!(foot_link, ContactPoint(cp, contactmodel))
                end
            end
        end
    end
    # Configuration of the ground
    if !symbolic && add_flat_ground
        frame = root_frame(mechanism)
        ground =
            HalfSpace3D(Point3D(frame, 0.0, 0.0, 0.0), FreeVector3D(frame, 0.0, 0.0, 1.0))
        add_environment_primitive!(mechanism, ground)
    end
    return mechanism
end

"""

Set the actual robot `mechansim` to his nominal state 
"""
function set_nominal!(
    rs::RobotSimulator,
    vis::MechanismVisualizer,
    boom::VecOrMat,
    actuators::VecOrMat,
    foot::VecOrMat,
)
    config = vec(reduce(hcat, [boom; actuators; foot]))
    set_configuration!(rs.state, config)
    zero_velocity!(rs.state)
    set_configuration!(vis, configuration(rs.state))
    return update_visulizer!(rs, vis)
end

"""

Set the actual robot `mechansim` to his initial state 
"""
function set_initialbody!(rs::RobotSimulator, vis::MechanismVisualizer)
    boom = [0, 0]
    foot = [0, 0]
    actuators = [0, 0, 0, 0]
    config = vec(reduce(hcat, [boom; actuators; foot]))
    set_configuration!(rs.state, config)
    zero_velocity!(rs.state)
    return update_visulizer!(rs, vis)
end

"""

Return the visualiser 
"""
function set_visulalizer(; mechanism::Mechanism, fileName::String = "ZMP_2DBipedRobot.urdf")
    urdfpath() = joinpath(packagepath(), fileName)
    vis = MechanismVisualizer(mechanism, URDFVisuals(urdfpath()))
    return vis
end

"""

Update the visualiser to the new state 
"""
function update_visulizer!(rs::RobotSimulator, vis::MechanismVisualizer)
    return set_configuration!(vis, RigidBodyDynamics.configuration(rs.state))
end

"""

Show the joint frame in the visualiser 
"""
function show_frame!(rs::RobotSimulator, vis::MechanismVisualizer)
    # Show the frame of each bodies
    robot_bodies = RigidBodyDynamics.bodies(rs.mechanism)
    for body in robot_bodies
        frame = RigidBodyDynamics.default_frame(body)
        setelement!(vis, frame)
    end
end

"""

Define the controller which will be used in the RigidBodyDynamics.simulate function 
"""
function trajectory_controller!(
    rs::RobotSimulator,
    time::Union{Vector, StepRangeLen},
    qref::Matrix,
    Δt::Float64,
    Kp::Float64,
    Ki::Float64,
    Kd::Float64,
    filename::String,
    write_torques::Bool = false,
)
    mechanism = rs.mechanism
    state = rs.state

    sim_index = 0
    index = 1

    ∫edt = zeros(4)
    pid = PID(Kp, Ki, Kd)
    temp = [0.0, 0.0, 0.0, 0.0]

    function controller!(τ, t, state)
        ddl = 2 # For 2 non-actuated foot 
        stop = false
        
        if (index >= size(qref)[1])
            stop = true
            index = size(qref)[1]
        else
            stop = false
        end
        v̇ = copy(velocity(state))
        actual_q = configuration(state)[(end - 3 - ddl):(end - ddl)]
        desired_q = vec(qref[index, :])
        Δq = desired_q - actual_q
        Δq̇ = vec(zeros(4, 1) .- velocity(state)[(end - 3 - ddl):(end - ddl)])

        v̇_new = pid_control!(pid, Δq, Δq̇, ∫edt, Δt)
        v̇[(end - 3 - ddl):(end - ddl)] .= v̇_new

        # Reset torque
        for joint in joints(mechanism)
            τ[velocity_range(state, joint)] .= 0 # no control
        end
        # Torque in the constrained space 
        newτ = inverse_dynamics(state, v̇) #- dynamics_bias(state)
        
        if (t >= sim_index * Δt && t < time[end])
            if(write_torques)
                data = [t,newτ[(end - 3 - ddl):(end - ddl)]...]
                open(filename, "a") do file
                    write(file, join(data, " ") * "\n")
                end
            end
            temp .= newτ[(end - 3 - ddl):(end - ddl)]
            sim_index = sim_index + 1
        end
        τ[(end - 3 - ddl):(end - ddl)] .= temp
        if (t >= time[index] && stop == false)
            index = index + 1
        end
        return nothing
    end
end

"""

Define the controller which will be used in the RigidBodyDynamics.simulate function 
"""
function controller_torque_input_file(
    rs::RobotSimulator,
    time::Float64,
    Δt::Float64,
    filename::String,
)
    state = rs.state
    sim_index = 0

    temp_τ = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    function controller!(τ, t, state)
        ddl = 2 # For 2 non-actuated foot 

        if (t >= sim_index * Δt && t < time)
            open(filename, "r") do file
                lines = readlines(file)                          
                if (sim_index < length(lines))                      
                    line = split(lines[sim_index+1] , " ")                                 
                    temp_τ[(end - 3 - ddl):(end - ddl)] .= parse.(Float64, line[2:end])                                         
                end
            end
            sim_index += 1
        end
        τ .= temp_τ

        return nothing
    end
end

"""

Estiiate the ZMP of the Virtual Robot 
"""

function measureZMP(
    rs::RobotSimulator,
    dynamics_results::DynamicsResult,
    state::MechanismState,
)
    RigidBodyDynamics.contact_dynamics!(dynamics_results, state)
    foot_link = findbody(rs.mechanism, "r_foot_link")
    sensor_right = RigidBodyDynamics.contact_wrench(dynamics_results, foot_link)
    foot_link = findbody(rs.mechanism, "l_foot_link")
    sensor_left = RigidBodyDynamics.contact_wrench(dynamics_results, foot_link)
    contact_torque_right = sensor_right.angular
    contact_force_right = sensor_right.linear
    contact_torque_left = sensor_left.angular
    contact_force_left = sensor_left.linear

    d = 0.0045 # half foot height
    pr_x_right =
        (-contact_torque_right[2] - contact_force_right[1] * d) / contact_force_right[3]
    pr_y_right =
        (contact_torque_right[1] - contact_force_right[2] * d) / contact_force_right[3]

    pr_x_left =
        (-contact_torque_left[2] - contact_force_left[1] * d) / contact_force_left[3]
    pr_y_left = (contact_torque_left[1] - contact_force_left[2] * d) / contact_force_left[3]

    px =
        (pr_x_right * contact_force_right[3] + pr_x_left * contact_force_left[3]) /
        (contact_force_right[3] + contact_force_left[3])
    py =
        (pr_y_right * contact_force_right[3] + pr_y_left * contact_force_left[3]) /
        (contact_force_right[3] + contact_force_left[3])
    if (contact_force_left[3] <= 0 && contact_force_right[3] >= 0)
        p = [pr_x_right, pr_y_right]
    elseif (contact_force_left[3] >= 0 && contact_force_right[3] <= 0)
        p = [pr_x_left, pr_y_left]
    else
        p = [px, py]
    end
    push!(rs.ZMP, p)
    return p
end