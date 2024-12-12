using Plots
using LaTeXStrings

F1 = true
F2 = true
F3 = true

include(joinpath(@__DIR__, "utils.jl"))

# FLODER 1: Walk_1_Slow
if(F1)
    # Input data files
    data_in_position = joinpath(@__DIR__, "..", "data", "inputs", "Position_20s.txt")
    data_in_velocity = joinpath(@__DIR__, "..", "data", "inputs", "V_20s.txt")
    data_in_PWM = joinpath(@__DIR__, "..", "data",  "inputs", "PWM_20s.txt")

    # Output data files
    data_out_position = joinpath(@__DIR__, "..", "data", "inputs", "Position.txt")
    data_out_velocity = joinpath(@__DIR__, "..", "data", "inputs","Velocity.txt")
    data_out_PWM = joinpath(@__DIR__, "..", "data", "inputs", "PWM.txt")
    data_out_Current = joinpath(@__DIR__, "..", "data", "outputs", "Current.txt")
    data_out_Torque = joinpath(@__DIR__, "..", "data", "outputs", "Torque.txt")
    data_out_extended_Torque = joinpath(@__DIR__, "..", "data", "outputs", "Torques_LabV.txt")
    data_out_permutated_Torque = joinpath(@__DIR__, "..", "data", "outputs", "Torques_simu.txt")
    data_out_opt_Torque = joinpath(@__DIR__, "..", "data", "outputs", "Torque_opt.txt")
    data_out_opt_extended_Torque = joinpath(@__DIR__, "..", "data", "outputs", "Torque_opt_LabV.txt")
    data_out_opt_permutated_Torque = joinpath(@__DIR__, "..", "data", "outputs", "Torques_opt_simu.txt")

    # File details
    freq = 20.0                                         # Frequency
    interval = (0.0,20.0)                               # Plot interval
    permutation = [(1,1,1.0),(2,2,-1.0),                # [t,HL,KL,HR,KR] (LabView) -> [t,HL,HR,FL,FR] (Code)
                    (3,4,-1.0),(4,3,1.0),(5,5,1.0)]     # H = Hip, K = Knee, R = Right, L = Left, t = Time
    Δt = 0.05                                           # 1/Frequency
    extension_factor = 50                               # Padding between two values
    max_lines = 20001

    remove_input_files          = false  # Remove raw data files (Current.txt, Torque.txt)
    process_input_files_LabView = false  # Generate Position.txt, Velocity.txt and PWM.txt
    convert_to_torque           = true   # Generate Current.txt, 
    extend_the_data             = true   # Allows for padding up to max_lines elements
    permute_columns             = true   # Enable column permutation (simu and robot coherence)
    to_plot                     = true   # Enables plots

    # File Operations
    if(process_input_files_LabView)
        process_lines(data_in_position, data_out_position, transform_position, freq, false)
        process_lines(data_in_velocity, data_out_velocity, transform_velocity, freq, false)
        process_lines(data_in_PWM, data_out_PWM, transform_PWM, freq, false)
    end

    if(convert_to_torque)
        convert_data(data_out_PWM, data_out_velocity, data_out_Current, convert_to_current_basic_model, false, false)
        convert_data(data_out_Current, data_out_velocity, data_out_Torque, convert_to_torque_basic_model, remove_input_files, false)
        convert_data(data_out_PWM,data_out_velocity,data_out_opt_Torque,convert_to_torque_optimised_model, false, false)
    end
    if(extend_the_data)
        extend_data(data_out_Torque, data_out_extended_Torque,Δt,extension_factor,max_lines = max_lines, remove_input_files)
        extend_data(data_out_opt_Torque, data_out_opt_extended_Torque,Δt,extension_factor,max_lines = max_lines, remove_input_files) 
    end
    if(permute_columns)
        column_permutation(data_out_extended_Torque,data_out_permutated_Torque,permutation, remove_input_files)
        column_permutation(data_out_opt_extended_Torque,data_out_opt_permutated_Torque,permutation, remove_input_files)
    end

    # Figure plotting
    if(to_plot)
        plot_data(data_out_position, "Position",interval)
        plot_data(data_out_permutated_Torque, "Perm_Torque",interval)
        plot_data(data_out_velocity, "Velocity",interval)
        plot_data(data_out_PWM, "PWM",interval)
        plot_data(data_out_opt_permutated_Torque,"Opt_Perm_Torque",interval)
    end
end

# FOLDER 2: Xing
if(F2)
    # Input data files
    data_Xing_slow = joinpath(@__DIR__, "..", "data", "outputs", "Torques_Xing_slow.txt")
    data_Xing_default = joinpath(@__DIR__, "..", "data", "outputs", "Torques_Xing_default.txt")

    # file details
        interval = (0.0,20.0)
        to_plot = true
        subsampling_factor = 2   # Keeps 1/x lines
        subsample = true         # Enables subsampling 

    # Figure plotting
    if (to_plot)
        plot_data(data_Xing_slow, "Torques_Xing slow",interval)
        plot_data(data_Xing_default, "Torques_Xing_default",interval) 
    end
end

# FOLDER 3: Walking_Patterns
if(F3)
    # Input data
    data_WP_slow = joinpath(@__DIR__, "..", "simulator", "walkingPattern_ref_short.csv")
    data_WP_slow_subsamp = joinpath(@__DIR__, "..", "simulator", "walkingPattern_ref_subsampled.csv")

    if(subsample)
        subsample_csv(data_WP_slow, 2, data_WP_slow_subsamp)
    end
end