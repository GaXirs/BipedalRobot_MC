# Load the URDF from the current folder
urdfpath()= joinpath(pwd(), "ZMP_2DBipedRobot_nodamping.urdf")
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

# Function to compute distances between consecutive bodies' frames
function compute_body_distances(mechanism, state)
    bodies = RigidBodyDynamics.bodies(mechanism)
    distances = []
    body_names = []

    for i in 1:(length(bodies) - 1)
        body1 = bodies[i]
        body2 = bodies[i + 1]
        
        # Get the frames of each body
        frame1 = RigidBodyDynamics.default_frame(body1)
        frame2 = RigidBodyDynamics.default_frame(body2)
        
        # Get the positions of the frames in the given configuration (state)
        position1 = RigidBodyDynamics.position(state, frame1)
        position2 = RigidBodyDynamics.position(state, frame2)
        
        # Compute the Euclidean distance between the frame positions
        dist = norm(position1 - position2)
        
        # Store body names and distances
        push!(body_names, (RigidBodyDynamics.name(body1), RigidBodyDynamics.name(body2)))
        push!(distances, dist)
    end
    
    # Create a DataFrame to show body names and distances
    df = DataFrame("Body 1" => first.(body_names), 
                   "Body 2" => last.(body_names), 
                   "Distance (m)" => distances)
    return df
end

# Compute distances between consecutive body frames
distances_df = compute_body_distances(mechanism, state)

# Display the table
println(distances_df)

