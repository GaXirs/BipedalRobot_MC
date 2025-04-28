using DelimitedFiles  
using Plots           
using Statistics

#----------------------------------------------------------------------------
#                       Code PARAMETERS
#----------------------------------------------------------------------------
File = "Voltage" # Position, Velocity, Voltage, Torque, Current
URDF = "Hybrid" # Prismatic, Hybrid
file_frequency  = 10000  # Sampling rate of the high-frequency signal (Hz)
goal_frequency = 200   # Sampling rate of the low-frequency signal (Hz) /!\ has to be a divider of file_frequency
duration = 20  # Duration of the signals (seconds)

Simulation_signal = joinpath(@__DIR__,"..","data","simulation", URDF, File * ".txt")

#----------------------------------------------------------------------------
#                       Code VARIABLES
#----------------------------------------------------------------------------
save_folder = joinpath(@__DIR__, "..", "data", "simulation", URDF, "moving_averaged", File * ".txt")
data = ["", "Left Hip", "Right Hip", "Left Knee", "Right Knee"]

if(File == "Position")
    high_freq_signal = readdlm(Simulation_signal)  # 10kHz signal
else
    high_freq_signal = readdlm(Simulation_signal)  # 10kHz signal
end

# Generate time arrays
t_low = 0:1/goal_frequency:duration  
t_high = 0:1/file_frequency:duration 

# Ensure the signals match the length of their time arrays
@assert size(high_freq_signal, 1) == length(t_high) "High-frequency signal length mismatch."

# Set the window size for moving average (based on 50Hz signal)
ma_window_size = round(Int, file_frequency / goal_frequency)

#----------------------------------------------------------------------------
#                       Under Sampling FUNCTION
#----------------------------------------------------------------------------

# Define Moving Average function manually
function moving_average(signal, window_size)
    # Initialize the filtered signal array
    filtered_signal = zeros(length(signal))

    # Iterate over the signal and compute the moving average
    for i in window_size:length(signal)
        filtered_signal[i] = mean(signal[i-window_size+1:i])  # Average over the window
    end
    
    return filtered_signal
end

# Resample the signal by taking the closest point from the high-frequency signal
function resample_signal_closest(signal, t_original, t_target)
    resampled_signal = zeros(length(t_target))
    
    for i in 1:length(t_target)
        # Find the index of the closest time point in the original signal
        closest_idx = argmin(abs.(t_original .- t_target[i]))
        resampled_signal[i] = signal[closest_idx]
    end
    
    return resampled_signal
end

#----------------------------------------------------------------------------
#                       ANALYSIS FUNCTION
#----------------------------------------------------------------------------

function calculate_me(reference::Vector{}, predicted::Vector{})
    me = mean(reference .- predicted)
    return me
end

function calculate_rmse(reference::Vector{}, predicted::Vector{})
    mse = mean((reference .- predicted).^2)
    rmse =  sqrt(mse)
    return rmse
end

#----------------------------------------------------------------------------
#                       CODE
#----------------------------------------------------------------------------

to_save = zeros(duration*goal_frequency+1,size(high_freq_signal,2))
to_save[:,1] = t_low

ref_signal = readdlm(joinpath(@__DIR__, "..", "data", "Robot_200Hz", "Inputs", File * ".txt"))
hybrid_cols = [1, 3, 2, 5, 4]
# Iterate over each data column
for col in 2:size(high_freq_signal, 2)  # Iterate over each data column

    # Apply Moving Average manually
    ma_filtered = moving_average(high_freq_signal[:, hybrid_cols[col]], ma_window_size)

    # Resample the filtered signals to the 50Hz low-frequency time points using the closest point method
    ma_resampled = resample_signal_closest(ma_filtered, t_high, t_low).*(-1.0)

    
    to_save[:,col] = ma_resampled
    
    plt = plot(t_low, ref_signal[:,col], label = "Reference signal", lw = 2, xlims=(0, 5))
    plot!(t_low, ma_resampled, label = "Simulation resampled", lw = 2, xlims=(0, 5)) 

    name = data[col]
    mean_error = calculate_me(ref_signal[:,col], ma_resampled)    
    RMSE = calculate_rmse(ref_signal[:,col], ma_resampled) 
    println("Mean error on $File $name : $mean_error")
    println("RMSE on $File $name : $RMSE")

    println()

    if(true)
        # Save the figure to verify moving average
        savefig(plt, joinpath(@__DIR__, "signal_simuvsrobot_$File$name.pdf"))
    end
end

#writedlm(save_folder, to_save, ' ')
