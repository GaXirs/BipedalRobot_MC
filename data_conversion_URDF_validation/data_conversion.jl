using Plots
using LaTeXStrings
include("utils_conversion.jl")

#----------------------------------------------------------------------------
#                       Folder DETAILS
#----------------------------------------------------------------------------

F1 = false
f1 = "data_conv_test"

F2 = false
f2 = "WP_straightline"

F3 = false
f3 = "WP_straightline_legswitched"

F4 = false
f4 = "Super_slow"

FSimu = true
torque_model = 0
fS_1 = "Easiest_model"
fS_2 = "Basic_model"
fS_3 = "Opt_model"

FWP = false
fWP = "WalkingPattern"

#----------------------------------------------------------------------------
#                       FILE DETAILS
#----------------------------------------------------------------------------
freq = 10000.0            # Frequency of measurements
interval = (0.0,5.0)      # Plot interval
# [t,HL,KL,HR,KR] (LabView) -> [t,HL,HR,FL,FR] (Code)
# H = Hip, K = Knee, R = Right, L = Left, t = Time
permutation = [(1,1,1.0),(2,2,1.0), (3,4,1.0),(4,3,-1.0),(5,5,-1.0)]                                                                   
Δt = 0.02               # 1/freq
extension_factor = 20   # Padding between two values
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
    raw_current  = joinpath(path, "Raw", "Current.txt")

 # Input data files
    in_position = joinpath(path, "Inputs", "Position.txt")
    in_velocity = joinpath(path, "Inputs", "Velocity.txt")
    in_voltage  = joinpath(path, "Inputs", "Voltage.txt")
    in_current  = joinpath(path, "Inputs", "Current.txt")

    in_position_temp = joinpath(path, "Inputs", "Position_temp.txt")
    in_velocity_temp = joinpath(path, "Inputs", "Velocity_temp.txt")
    in_voltage_temp  = joinpath(path, "Inputs", "Voltage_temp.txt")
    in_current_temp  = joinpath(path, "Inputs", "Current_temp.txt")

 # Output data files
    out_current = joinpath(path, "Outputs", "Current.txt")
    out_torque_c_fl  = joinpath(path, "Outputs", "Torque_c_fl.txt")
    out_torque_c_bm  = joinpath(path, "Outputs", "Torque_c_bm.txt")
    #out_torque_c_om  = joinpath(path, "Outputs", "Torque_c_om.txt")

    out_torque_v_fl  = joinpath(path, "Outputs", "Torque_v_fl.txt")
    out_torque_v_bm  = joinpath(path, "Outputs", "Torque_v_bm.txt")
    out_torque_v_om  = joinpath(path, "Outputs", "Torque_v_om.txt")

 # Simulation data files
    simu_torque_c_bm = joinpath(path, "Simulations", "Torque_c_bm.txt")
    #simu_torque_c_om = joinpath(path, "Simulations", "Torque_c_om.txt")

    simu_torque_v_bm = joinpath(path, "Simulations", "Torque_v_bm.txt")
    simu_torque_v_om = joinpath(path, "Simulations", "Torque_v_om.txt")

 # Preprocessing (1/2)
    compute_transform(raw_position, in_position_temp, transform_position, freq)
    compute_transform(raw_velocity, in_velocity_temp, transform_velocity, freq)
    compute_transform(raw_voltage , in_voltage_temp , transform_voltage , freq)
    compute_transform(raw_current , in_current_temp , transform_current , freq)

 # Preprocessing (2/2)
    apply_permutation(in_position_temp, in_position, permutation, remove_temp_file)
    apply_permutation(in_velocity_temp, in_velocity, permutation, remove_temp_file)
    apply_permutation(in_voltage_temp , in_voltage , permutation, remove_temp_file)
    apply_permutation(in_current_temp , in_current , permutation, remove_temp_file)

 # Processing
    compute_model(in_voltage , in_velocity, out_current    , to_current_basic_model)
    compute_model(in_current , in_velocity, out_torque_c_fl, to_torque_frictionless_model)
    compute_model(out_current, in_velocity, out_torque_v_fl, to_torque_frictionless_model)
    compute_model(in_current , in_velocity, out_torque_c_bm, to_torque_basic_model)
    compute_model(out_current, in_velocity, out_torque_v_bm, to_torque_basic_model)
    compute_model(in_voltage , in_velocity, out_torque_v_om, to_torque_optimised_model)

 # Post-processing  
    extend_data(out_torque_c_bm, simu_torque_c_bm, Δt, extension_factor; max_lines = max_lines)
    #extend_data(out_torque_c_om, simu_torque_c_om, Δt, extension_factor; max_lines = max_lines)
    extend_data(out_torque_v_bm, simu_torque_v_bm, Δt, extension_factor; max_lines = max_lines)
    extend_data(out_torque_v_bm, simu_torque_v_om, Δt, extension_factor; max_lines = max_lines) 

    plot_path = joinpath(path, "Images")
    plot_data(in_position, plot_path, "Position"         ,interval)
    plot_data(in_current , plot_path, "Measured_Current" ,interval)
    plot_data(in_velocity, plot_path, "Velocity"         ,interval)
    plot_data(in_voltage , plot_path, "Voltage"          ,interval)
    plot_data(out_current, plot_path, "Computed_Current" ,interval)
    plot_data(out_torque_v_fl, plot_path, "Torque_v_fl"  ,interval)
    plot_data(out_torque_v_bm, plot_path, "Torque_v_bm"  ,interval)
    plot_data(out_torque_v_om, plot_path, "Torque_v_om"  ,interval)
    plot_data(out_torque_c_fl, plot_path, "Torque_c_fl"  ,interval)
    plot_data(out_torque_c_bm, plot_path, "Torque_c_bm"  ,interval)
    #plot_data(out_torque_c_om, plot_path, "Torque_v_om"  ,interval)
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
if(F4)
   folder_full_process(f4)
end
if(FSimu)
   if(torque_model == 0)
      path = joinpath(@__DIR__, "..", "data", "simulation", fS_1)
   elseif(torque_model == 1)
      path = joinpath(@__DIR__, "..", "data", "simulation", fS_2)
   else
      path = joinpath(@__DIR__, "..", "data", "simulation", fS_3)
   end
   plot_data(joinpath(path, "Outputs", "Torque.txt"), joinpath(path, "Images"), "Torque", interval)
   plot_data(joinpath(path, "Outputs", "Current.txt"), joinpath(path, "Images"), "Current", interval)
   plot_data(joinpath(path, "Outputs", "Velocity.txt"), joinpath(path, "Images"), "Velocity", interval)
   plot_data(joinpath(path, "Outputs", "Voltage.txt"), joinpath(path, "Images"), "Voltage", interval)
   plot_data(joinpath(path, "Outputs", "Position.txt"), joinpath(path, "Images"), "Position", interval)
end
if(FWP)
   path = joinpath(@__DIR__, "..", "data", fWP)
   plot_data(joinpath(path, "Outputs", "Torque.txt"), joinpath(path, "Images"), "Torques_Xing", interval)
end
#----------------------------------------------------------------------------
