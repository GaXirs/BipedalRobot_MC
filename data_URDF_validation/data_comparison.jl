using DelimitedFiles  
using Plots           
using Statistics

Robot_signal = joinpath(@__DIR__,"..","data","WP_validation", "Inputs", "Voltage.txt")
Simulation_signal = joinpath(@__DIR__,"..","data","simulation", "Opt_model", "Outputs", "Voltage.txt")

# Load the signals from the text files
low_freq_signal = readdlm(Robot_signal)  # 50Hz signal
high_freq_signal = readdlm(Simulation_signal)  # 10kHz signal

# Define parameters of the signals
low_freq_sampling_rate = 50   # Sampling rate of the low-frequency signal (Hz)
high_freq_sampling_rate = 10000  # Sampling rate of the high-frequency signal (Hz)
duration = 20  # Duration of the signals (seconds)

# Generate time arrays
t_low = 0:1/low_freq_sampling_rate:duration  
t_high = 0:1/high_freq_sampling_rate:duration 

# Ensure the signals match the length of their time arrays
@assert size(low_freq_signal, 1) == length(t_low) "Low-frequency signal length mismatch."
@assert size(high_freq_signal, 1) == length(t_high) "High-frequency signal length mismatch."

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

# Define Exponential Moving Average function
function exponential_moving_average(signal, alpha)
    ema = zeros(length(signal))
    ema[1] = signal[1]
    for i in 2:length(signal)
        ema[i] = alpha * signal[i] + (1 - alpha) * ema[i-1]
    end
    return ema
end

# Set the window size for moving average (based on 50Hz signal)
ma_window_size = round(Int, high_freq_sampling_rate / low_freq_sampling_rate)

# Set the smoothing factor for the Exponential Moving Average
alpha = 2 / (ma_window_size*0.02 + 1)  # typical value for EMA

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

# Iterate over each data column
for col in 2:size(low_freq_signal, 2)  # Iterate over each data column

    # Apply Moving Average manually
    ma_filtered = moving_average(high_freq_signal[:, col], ma_window_size)

    # Apply Exponential Moving Average
    ema_filtered = exponential_moving_average(high_freq_signal[:, col], alpha)

    # Resample the filtered signals to the 50Hz low-frequency time points using the closest point method
    ma_resampled = resample_signal_closest(ma_filtered, t_high, t_low)
    ema_resampled = resample_signal_closest(ema_filtered, t_high, t_low)

    # Plot the signals: original 10kHz, Moving Average and EMA filtered (downsampled)
    plt = plot(
        t_high, high_freq_signal[:, col], label = "Original High-Frequency Signal (10kHz)",
        xlabel = "Time (s)", ylabel = "Voltage",
        title = "Signal Comparison (Data $col)", lw = 0.1  # Finer lines
    )
    
    # Plot Moving Average filtered signal
    plot!(t_low, ma_resampled, label = "Moving Average Filtered (50Hz)", lw = 0.1)  # Finer lines

    # Plot Exponential Moving Average filtered signal
    plot!(t_low, ema_resampled, label = "EMA Filtered (50Hz)", lw = 0.1)  # Finer lines

    # Save the figure
    savefig(plt, joinpath(@__DIR__, "Images","Comparison", "Filtered_Signals", "signal_comparison_data_$col.pdf"))

    # Optionally: Calculate error, NRMSE, and standard deviation of error
    error_ma = low_freq_signal[:, col] - ma_resampled
    error_ema = low_freq_signal[:, col] - ema_resampled

    nrmse_ma = sqrt(mean(error_ma.^2)) / (maximum(low_freq_signal[:, col]) - minimum(low_freq_signal[:, col]))
    std_error_ma = std(error_ma)
    mean_error_ma = mean(error_ma)

    nrmse_ema = sqrt(mean(error_ema.^2)) / (maximum(low_freq_signal[:, col]) - minimum(low_freq_signal[:, col]))
    std_error_ema = std(error_ema)
    mean_error_ema = mean(error_ema)

    # Print or log the results for both filters
    println("Data Column $col:")
    println("  Moving Average NRMSE: ", nrmse_ma)
    println("  Moving Average Std Error: ", std_error_ma)
    println("  Mean Error: ", mean_error_ma)
    println("  EMA NRMSE: ", nrmse_ema)
    println("  EMA Std Error: ", std_error_ema)
    println("  Mean Error: ", mean_error_ema)

    # Plot the low-frequency signal as a function of the high-frequency signal
    plt = plot(
        low_freq_signal[:, col], ma_resampled, seriestype = :scatter,
        xlabel = "High-Frequency Signal (10kHz)", ylabel = "Low-Frequency Signal (50Hz)",
        title = "Low-Frequency Signal vs High-Frequency Signal (Data $col)"
    )

    # Add the function f(x) = x to the plot
    plot!(low_freq_signal[:, col], low_freq_signal[:, col], label = "f(x) = x", lw = 2, color = :red)

    # Save the robot signal and the interpolated simulation signal figures
    savefig(plt, joinpath(@__DIR__, "Images", "Comparison", "signal_comparison_data_$col.png"))
end