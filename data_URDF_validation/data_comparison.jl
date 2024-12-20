using DelimitedFiles  
using Plots           
using Interpolations
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

# Interpolate the high-frequency signals at the low-frequency time points
for col in 2:size(low_freq_signal, 2)  # Iterate over each data column

    interp_high = LinearInterpolation(t_high, high_freq_signal[:, col])  # Interpolate high-frequency signal
    high_resampled = interp_high.(t_low)  # Resample high-frequency signal at low-frequency time points

    # Plot the low-frequency signal
    plt = plot(
        t_low, low_freq_signal[:, col], label = "Low-Frequency Signal (50Hz)",
        xlabel = "Time (s)", ylabel = "Voltage",
        title = "Low-Frequency Signal (Data $col)",
        lw = 2
    )
    # Plot the high-resampled signal
    plot!(
        t_low, high_resampled, label = "High-Frequency Resampled Signal (10kHz->50Hz)",
        lw = 2
    )
    """
    # If you want to see the difference between the resampled and original signal
    # NB : there is no noticeable difference
    plot!(
        t_high, high_freq_signal[:,col], label = "High-Frequency Signal (10kHz)",
        lw = 2
    )
    """

    # Save the Interpolation figures
    savefig(plt, joinpath(@__DIR__, "Images", "Simulation_interpolation", "signal_data_$col.png"))
    
    # Plot the high-frequency signal as a function of the low-frequency signal
    plt2 = plot(
        low_freq_signal[:, col], high_resampled, seriestype = :scatter,
        xlabel = "Low-Frequency Signal (50Hz)", ylabel = "High-Frequency Signal (10kHz)",
        title = "High-Frequency Signal vs Low-Frequency Signal (Data $col)"
    )
    # Add the function f(x) = x to the plot
    plot!(low_freq_signal[:, col], low_freq_signal[:, col], label = "f(x) = x", lw = 2, color = :red)

    # Save the robot signal and the interpolated simulation signal figures
    savefig(plt2, joinpath(@__DIR__, "Images", "Comparison", "signal_comparison_data_$col.png"))
    
    # Calculate error, NRMSE, and standard deviation
    error = low_freq_signal[:, col] - high_resampled
    nrmse = sqrt(mean(error.^2)) / (maximum(low_freq_signal[:, col]) - minimum(low_freq_signal[:, col]))
    std_error = std(error)
    mean_error = mean(error)

    # Print or log the results
    println("Data Column $col:")
    println("NRMSE: ", nrmse)
    println("Standard Deviation of Error: ", std_error)
    println("Mean Error: ", mean_error)
    println("")
end
