using Printf
# This code is used to determine what would be a reasonnable range for the motor positions


# Function to calculate alpha and X
function compute_geometry_vertical(Lt, Ll, theta)
    # Convert theta to radians
    theta_rad = deg2rad(theta)

    # Compute alpha using the law of cosines
    alpha_rad = acos((Ll + Lt * cos(theta_rad)) / (Lt + Ll))
    alpha_deg = rad2deg(alpha_rad)  # Convert to degrees

    # Compute X (horizontal distance)
    X = sin(alpha_rad) * (Lt + Ll) + sin(theta_rad) * Lt

    return alpha_deg, X
end

# Function to calculate alpha and X
function compute_geometry_non_vertical(Lt, Ll, thetaH, thetaK)
    # Idea: robot leg has 2 degrees more
    # Convert theta to radians
    thetaH_rad = deg2rad(thetaH)
    thetaK_rad = deg2rad(thetaK)

    # Compute alpha using the law of cosines
    alpha_rad = acos((Ll*cos(thetaK_rad) + Lt * cos(thetaH_rad)) / (Lt + Ll))
    alpha_deg = rad2deg(alpha_rad)  # Convert to degrees

    # Compute X (horizontal distance)
    X = sin(alpha_rad) * (Lt + Ll) + sin(thetaH_rad) * Lt - sin(thetaK_rad) * Ll

    return alpha_deg, X
end

# Robot geometrical parameters
Lt = 0.20125  # Length of the thigh
Ll = 0.172   # Length of the lower leg

@printf "########################################## \n"
@printf "############## Vertical leg ############## \n"
@printf "########################################## \n"
@printf "\n"
theta = 10  # Bent knee angle in degrees

alpha, X = compute_geometry_vertical(Lt, Ll, theta)

# Print results
@printf "Alpha (hip angle) = %.3f degrees\n" alpha
@printf "Distance between feet (X) = %.3f meters\n" X
@printf "\n"

theta = 8
alpha, X = compute_geometry_vertical(Lt, Ll, theta)

# Print results
@printf "Alpha (hip angle) = %.3f degrees\n" alpha
@printf "Distance between feet (X) = %.3f meters\n" X

@printf "\n"
@printf "########################################## \n"
@printf "############ Non-Vertical leg ############ \n"
@printf "########################################## \n"
@printf "\n"

thetaH = 10  # Bent knee angle in degrees
thetaK = 12

alpha, X = compute_geometry_non_vertical(Lt, Ll, thetaH, thetaK)

# Print results
@printf "Alpha (hip angle) = %.3f degrees\n" alpha
@printf "Distance between feet (X) = %.3f meters\n" X
@printf "\n"

thetaH = 10.0
thetaK = 11.0
alpha, X = compute_geometry_non_vertical(Lt, Ll, thetaH, thetaK)

# Print results
@printf "Alpha (hip angle) = %.3f degrees\n" alpha
@printf "Distance between feet (X) = %.3f meters\n" X
