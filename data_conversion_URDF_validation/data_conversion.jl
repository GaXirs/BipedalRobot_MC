using Plots
using LaTeXStrings

include(joinpath(@__DIR__, "utils.jl"))

# Data files to be preocessed
data_in_position = joinpath(@__DIR__, "..", "data", "Position_20s.txt")
data_in_velocity = joinpath(@__DIR__, "..", "data", "V_20s.txt")
data_in_PWM = joinpath(@__DIR__, "..", "data", "PWM_20s.txt")

# Output data files
data_out_position = joinpath(@__DIR__, "..", "data", "Position.txt")
data_out_velocity = joinpath(@__DIR__, "..", "data", "Velocity.txt")
data_out_PWM = joinpath(@__DIR__, "..", "data", "PWM.txt")
data_out_Current = joinpath(@__DIR__, "..", "data", "Current.txt")
data_out_Torque = joinpath(@__DIR__, "..", "data", "Torque.txt")
data_out_extended_Torque = joinpath(@__DIR__, "..", "data", "Torques_LabV.txt")
data_out_permutated_Torque = joinpath(@__DIR__, "..", "data", "Torques_simu.txt")
data_Xing = joinpath(@__DIR__, "..", "data", "Torques_Xing_slow.txt")
data_out_opt_Torque = joinpath(@__DIR__, "..", "data", "Torque_opt.txt")
data_out_opt_extended_Torque = joinpath(@__DIR__, "..", "data", "Torque_opt_LabV.txt")
data_out_opt_permutated_Torque = joinpath(@__DIR__, "..", "data", "Torques_opt_simu.txt")
# frequency
freq = 20.0

remove_input_files = true # remove raw data files (Current.txt, Torque.txt)
process_input_files_LabView = false # To generate Position.txt, Velocity.txt and PWM.txt
convert_to_torque = true

if(process_input_files_LabView)
    process_lines(data_in_position, data_out_position, transform_position, freq, false)
    process_lines(data_in_velocity, data_out_velocity, transform_velocity, freq, false)
    # Note if you are actually reading the velocity output file :
    # The robot first bends its knees, then marks a stop, then walks
    # It is thus normal to have values at 0 from ~2.5 - ~5 [s]
    process_lines(data_in_PWM, data_out_PWM, transform_PWM, freq, false)
end

if(convert_to_torque)
    convert_data(data_out_PWM, data_out_velocity, data_out_Current, convert_to_current_basic_model, false, false)
    convert_data(data_out_Current, data_out_velocity, data_out_Torque, convert_to_torque_basic_model, remove_input_files, false)
    extend_data(data_out_Torque, data_out_extended_Torque,0.05,50,max_lines = 20001, remove_input_files)

    convert_data(data_out_PWM,data_out_velocity,data_out_opt_Torque,convert_to_torque_optimised_model, false, false)
    extend_data(data_out_opt_Torque, data_out_opt_extended_Torque,0.05,50,max_lines = 20001, remove_input_files) 
end
permutation = [(1,1,1.0),(2,2,-1.0),(3,4,-1.0),(4,3,1.0),(5,5,1.0)] # [HL,KL,HR,KR] (LabView) -> [HL,HR,FL,FR] (Code)
column_permutation(data_out_extended_Torque,data_out_permutated_Torque,permutation, remove_input_files)
column_permutation(data_out_opt_extended_Torque,data_out_opt_permutated_Torque,permutation, remove_input_files)

interval = (0.0,20.0)
plot_data(data_out_position, "Position",interval)
plot_data(data_out_permutated_Torque, "Perm_Torque",interval)
plot_data(data_out_velocity, "Velocity",interval)
plot_data(data_out_PWM, "PWM",interval)
plot_data(data_Xing, "Torques_Xing",interval)
plot_data(data_out_opt_permutated_Torque,"Opt_Perm_Torque",interval)