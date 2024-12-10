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
data_out_extended_Torque = joinpath(@__DIR__, "..", "data", "Torques_LV.txt")
# frequency
freq = 20.0

remove_input_files = true
process_input_files_LabView = false

if(process_input_files_LabView)
    process_lines(data_in_position, data_out_position, transform_position, freq, remove_input_files)
    process_lines(data_in_velocity, data_out_velocity, transform_velocity, freq, remove_input_files)
    # Note if you are actually reading the velocity output file :
    # The robot first bends its knees, then marks a stop, then walks
    # It is thus normal to have values at 0 from ~2.5 - ~5 [s]
    process_lines(data_in_PWM, data_out_PWM, transform_PWM, freq, remove_input_files)
end

convert_data(data_out_PWM, data_out_velocity, data_out_Current, convert_to_current_basic_model, false, false)
convert_data(data_out_Current, data_out_velocity, data_out_Torque, convert_to_torque_basic_model, remove_input_files, false)
extend_data(data_out_Torque, data_out_extended_Torque,0.05,50,max_lines = 10000, remove_input_files)
