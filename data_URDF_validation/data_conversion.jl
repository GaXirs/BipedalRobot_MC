using Plots
using LaTeXStrings
using DelimitedFiles            
include("utils_conversion.jl")

#----------------------------------------------------------------------------
#                       Folder DETAILS
#----------------------------------------------------------------------------

F1 = false
f1 = "WP_straightline_intheair"

F2 = false
f2 = "WP_validation"

F3 = false
f3 = "WP_validation_200Hz"

FSimu = false

FDionysos = false
fSD = "Dionysos"

FWP = false
fWP = "WalkingPattern"

#----------------------------------------------------------------------------
#                       FILE DETAILS
#----------------------------------------------------------------------------
freq = 200.0            # Frequency of measurements
interval = (0.0,2.0)      # Plot interval
# [t,HL,KL,HR,KR] (LabView) -> [t,HL,HR,KL,KR] (Code)
# H = Hip, K = Knee, R = Right, L = Left, t = Time
permutation = [(1,1,1.0),(2,2,1.0), (3,4,1.0),(4,3,-1.0),(5,5,-1.0)]                                                                   
Δt = 1/freq                # 1/freq
extension_factor = 5   # Padding between two values
max_lines = 20001       # Limit the number of lines after padding
remove_temp_file = true # removes non permutated files
#----------------------------------------------------------------------------

#----------------------------------------------------------------------------
# Folder processers
#----------------------------------------------------------------------------
function folder_full_process(folder_name::String)

    path = joinpath(@__DIR__, "..", "data", folder_name)
 # Raw data files
    raw_position = joinpath(path, "Raw", "Position.txt")
    raw_velocity = joinpath(path, "Raw", "Velocity.txt")
    raw_voltage  = joinpath(path, "Raw", "Voltage.txt")

 # Input data files
    in_position = joinpath(path, "Inputs", "Position.txt")
    in_velocity = joinpath(path, "Inputs", "Velocity.txt")
    in_voltage  = joinpath(path, "Inputs", "Voltage.txt")

    in_position_temp = joinpath(path, "Inputs", "Position_temp.txt")
    in_velocity_temp = joinpath(path, "Inputs", "Velocity_temp.txt")
    in_voltage_temp  = joinpath(path, "Inputs", "Voltage_temp.txt")

    out_torque_v_om  = joinpath(path, "Outputs", "Torque.txt")

 # Simulation data files
    simu_torque_v_om = joinpath(path, "Simulations", "Torque.txt")

 # Preprocessing (1/2)
    compute_transform(raw_position, in_position_temp, transform_position, freq)
    compute_transform(raw_velocity, in_velocity_temp, transform_velocity, freq)
    compute_transform(raw_voltage , in_voltage_temp , transform_voltage , freq)

 # Preprocessing (2/2)
    apply_permutation(in_position_temp, in_position, permutation, remove_temp_file)
    apply_permutation(in_velocity_temp, in_velocity, permutation, remove_temp_file)
    apply_permutation(in_voltage_temp , in_voltage , permutation, remove_temp_file)

 # Processing
    compute_model(in_voltage , in_velocity, out_torque_v_om, to_torque_model)

 # Post-processing  
    extend_data(out_torque_v_om, simu_torque_v_om, Δt, extension_factor; max_lines = max_lines) 

    plot_path = joinpath(path, "Images")
    plot_data(in_position, plot_path, "Position"         ,interval)
    plot_data(in_velocity, plot_path, "Velocity"         ,interval)
    plot_data(in_voltage , plot_path, "Voltage"          ,interval)

    plot_data(out_torque_v_om, plot_path, "Torque"  ,interval)
    plot_data(simu_torque_v_om, plot_path, "Extended_torque", interval)

end
#----------------------------------------------------------------------------

#----------------------------------------------------------------------------
# FLODER PROCESSING
#----------------------------------------------------------------------------
if(F1)
   folder_full_process(f1)
end
if(F2)
   folder_full_process(f2)
end
if(F3)
   folder_full_process(f3)
end

if(FSimu)
   path = joinpath(@__DIR__, "..", "data", "simulation")
   plot_data(joinpath(path, "Outputs", "Torque.txt"), joinpath(path, "Images"), "Torque", interval)
   plot_data(joinpath(path, "Outputs", "Velocity.txt"), joinpath(path, "Images"), "Velocity", interval)
   plot_data(joinpath(path, "Outputs", "Voltage.txt"), joinpath(path, "Images"), "Voltage", interval)
   plot_data(joinpath(path, "Outputs", "Position.txt"), joinpath(path, "Images"), "Position", interval)
end

if(FDionysos)
   path = joinpath(@__DIR__, "..", "data", fSD)
   plot_data(joinpath(path, "Outputs", "Torque.txt"), joinpath(path, "Images"), "Torque", interval)
   plot_data(joinpath(path, "Outputs", "Velocity.txt"), joinpath(path, "Images"), "Velocity", interval)
   plot_data(joinpath(path, "Outputs", "Voltage.txt"), joinpath(path, "Images"), "Voltage", interval)
   plot_data(joinpath(path, "Outputs", "Position.txt"), joinpath(path, "Images"), "Position", interval)
end

if(FWP)
   path = joinpath(@__DIR__, "..", "data", fWP)
   plot_data(joinpath(path, "Outputs", "Torque.txt"), joinpath(path, "Images"), "Torques_Xing", interval)
end
#----------------------------------------------------------------------------

"""
simu_torque = joinpath(@__DIR__, "..", "data", "simulation", "Torque.txt")
velocity = joinpath(@__DIR__, "..", "data", "simulation", "No_damping", "Outputs", "Velocity.txt")
voltage = joinpath(@__DIR__, "..", "data", "simulation", "No_damping", "Outputs", "Voltage.txt")
compute_model(voltage, velocity, simu_torque, to_torque_model)

simu_signal = readdlm(simu_torque)
robot_signal = readdlm(joinpath(@__DIR__, "..", "data", "simulation", "No_damping", "Outputs", "Torque.txt"))

for col in 2:size(simu_signal, 2)  # Iterate over each data column
   # Plot the signals: Moving Average and Robot data
   plt = plot(
      simu_signal[:, 1], simu_signal[:, col], label = "Moving Averaged Simulation",
       xlabel = "Time (s)", ylabel = File,
       title = data[col] * " $File Comparison", lw = 2,
       xlims=(0, 5) 
   )
   
   plot!(robot_signal[:, 1], robot_signal[:, col], label = "Robot Output", lw = 2, xlims=(0, 5)) 

   # Save the figure
   savefig(plt, joinpath(@__DIR__, "Images", "signal_simuvsrobot_$File$col.pdf"))
end
"""