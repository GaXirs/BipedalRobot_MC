using LaTeXStrings

function toInt16(numbers::Vector{Int})::Vector{Int16}
    # We assume numbers are 16-bit unsigned integers (UInt16), reinterpret them to Int16
    return reinterpret(Int16, UInt16.(numbers))
end

function transform_position(numbers::Vector{Int16}, frequency::Float64)::Vector{Float64}
    # The first element of each line is the index of the data (beginning at 0)
    # Divide this value by the frequency to get time
    result = [numbers[1] / frequency]  
    
    # Transform to °
    # For the speed, the data is in [ticks]
    # The maximum value is 4095 ticks
    # Hence, to transform the data to °, just multiply it by (360/4095)
    append!(result, numbers[2:end] .*(360 / 4095))

    return result
end

function transform_velocity(numbers::Vector{Int16}, frequency::Float64)::Vector{Float64}
    # The first element of each line is the index of the data (beginning at 0)
    # Divide this value by the frequency to get time
    result = [numbers[1] / frequency]  
    
    # Transform to rad/s
    # The input data is an adimensionalisation of RPMs by 0.229rmp
    # To transform RMP into RPS (rotation per second), divide it by 60
    # Multiply then by 2π to get the rad/s
    append!(result, numbers[2:end] .*(0.229*2π / 60)) 
    
    return result
end

function transform_PWM(numbers::Vector{Int16}, frequency::Float64)::Vector{Float64}
    # The first element of each line is the index of the data (beginning at 0)
    # Divide this value by the frequency to get time
    result = [numbers[1] / frequency]  
    
    # Transform to PWM [%]
    # A PWM of 100% corresponf to 885 -> we just need to divide what we have by 885
    append!(result, numbers[2:end] .*(1 / 885))

    return result
end

function convert_to_current_basic_model(PWM::Vector{Float64}, RPS::Vector{Float64}):: Vector{Float64}
    # Motor Caracteristics: S. Deligne 
    Un  = 12          # Nominal tension [V]
    R   = 9.3756      # Armature resistance [Ω]
    HGR = 353.5       # Hip gear-ratio
    KGR = 212.6       # Knee gear-ratio
    kϕ  = 3.6103/HGR  # Back-EMF constant ke' [Nm*s/rad] (linked to joint speed)

    # Simple motor model : U = R*i + kϕ ω -> i = (U - kϕ ω)/R
    U = PWM[2:end] .* Un
    ω = RPS[2:end] .* [HGR, KGR, HGR, KGR] # ω = ̇q * GR
    i = (U .- (ω .* kϕ)) ./ R

    return append!([PWM[1]],i)
end

function convert_to_torque_basic_model(current::Vector{Float64}, RPS::Vector{Float64}):: Vector{Float64}
    # Motor Caracteristics: S. Deligne
    HGR = 353.5         # Hip gear-ratio
    KGR = 212.6         # Knee gear-ratio
    Kv  = 0.22/HGR      # Viscous friction constant [Nm*s/rad] (linked to joint speed)
    τc  = 0.128         # Dry friction torque [Nm]
    kϕ  = 3.6103/HGR    # Back-EMF constant ke' [Nm*s/rad] (linked to joint speed) 

    # Simple motor model : τ = kϕ*i - τc - Kv q̇
    ω = RPS[2:end] .* [HGR, KGR, HGR, KGR]
    τ_0 = current[2:end].* [HGR, KGR, HGR, KGR] .* kϕ .- ω .* Kv
    τ = ifelse.(τ_0 .> 0, max.(τ_0 .- τc, 0.0), min.(τ_0 .+ τc, 0.0)) # L1, L2, R1, R2
    return append!([current[1]],τ)
end

function convert_to_torque_optimised_model(PWM::Vector{Float64}, RPS::Vector{Float64}):: Vector{Float64}
    Un = 12.0
    U = PWM[2:end] .* Un
    # Motor Caracteristics: S. Deligne
    HGR = 353.5           # Hip gear-ratio
    KGR = 212.6           # Knee gear-ratio
    ktp  = 0.395/HGR      # Torque constant with respect to the voltage [Nm/V] 
    Kvp  = 1.589/HGR      # Viscous friction constant [Nm*s/rad] (linked to motor speed)
    τc  = 0.065           # Dry friction torque [Nm]

    # Simple motor model : τ = kt'*U - (τc + Kv'q̇) - C(q,q̇)
    ω = RPS[2:end] .* [HGR, KGR, HGR, KGR]
    τ_0 = U .* [HGR, KGR, HGR, KGR] .* ktp  .- ω .* Kvp
    τ = ifelse.(τ_0 .> 0, max.(τ_0 .- τc, 0.0), min.(τ_0 .+ τc, 0.0))
    
    return append!([PWM[1]],τ)
end
    
function process_lines(input_file::String, output_file::String, func::Function, frequency::Float64, remove_input_file::Bool)
    open(input_file, "r") do infile
        open(output_file, "w") do outfile
            for line in eachline(infile)
                # Replace commas with periods to standardize decimal notation
                line = replace(line, "," => ".")
                # Parse numbers from the line, round to Int, and process
                numbers = round.(Int, parse.(Float64, split(line)))  # Parse and round to integers
                # Transform from Int to Int16 (reinterpret) and then process
                int16_numbers = toInt16(numbers)
                # Apply the transformation function
                processed_numbers = func(int16_numbers, frequency)
                # Write results to output file
                write(outfile, join(processed_numbers, " "), "\n")
            end
        end
    end
    if(remove_input_file)
        rm(input_file)
    end 
end

function convert_data(input_file_1::String, input_file_2::String, output_file::String, func::Function, remove_input_file_1::Bool, remove_input_file_2::Bool)
    # Open all files
    open(input_file_1, "r") do f1
        open(input_file_2, "r") do f2
            open(output_file, "w") do out
                # Read lines from both input files simultaneously
                for (line1, line2) in zip(eachline(f1), eachline(f2))
                    # Parse floats from the lines
                    data1 = parse.(Float64, split(line1, " "))
                    data2 = parse.(Float64, split(line2, " "))
                    
                    # Apply the operation on the two arrays
                    result = func(data1, data2)
                    
                    # Write the result to the output file
                    write(out, join(result, " "), "\n")
                end
            end
        end
    end
    if(remove_input_file_1)
        rm(input_file_1)
    end
    if(remove_input_file_2)
        rm(input_file_2)
    end 
end

function extend_data(input_file::String, output_file::String, δt::Float64, ext_fact::Int, remove_file::Bool; max_lines::Union{Nothing, Int} = nothing)
    open(input_file, "r") do infile
        open(output_file, "w") do outfile
            total_lines_written = 0  # Counter for lines written to the output file

            for line in eachline(infile)
                # Stop if the total lines written exceeds max_lines
                if max_lines !== nothing && total_lines_written >= max_lines
                    break
                end
                # Parse the line into a vector of floats
                data = parse.(Float64, split(line, " "))
                # Extract the time value (first column)
                t = data[1]
                # Duplicate the line ext_fact times with adjusted time values
                for i in 0:(ext_fact - 1)
                    # Stop if the total lines written exceeds max_lines
                    if max_lines !== nothing && total_lines_written >= max_lines
                        break
                    end
                    # Compute the adjusted time
                    new_t = t + i * δt / ext_fact
                    # Write the new line to the output file
                    println(outfile, join(vcat([new_t], data[2:end]), " "))
                    # Increment the counter
                    total_lines_written += 1
                end
            end
        end
    end
    if(remove_file)
        rm(input_file)
    end
end

function column_permutation(input_file::String, output_file::String, permutation::Vector{Tuple{Int, Int,Float64}}, remove_input_files::Bool)
    # Open the input and output files
    open(input_file, "r") do infile
        open(output_file, "w") do outfile
            for line in eachline(infile)
                # Parse the input line into columns (assume space-separated values)
                columns = parse.(Float64, split(line, " "))
                # Create a new row with permuted columns
                permuted_row = zeros(Float64, length(columns))
                for (source_col, target_col,sign) in permutation
                    permuted_row[target_col] = columns[source_col]*sign
                end
                # Write the permuted row to the output file
                write(outfile, join(permuted_row, " ") * "\n")
            end
        end
    end
    if(remove_input_files)
        rm(input_file)
    end
end

function plot_data(file_path::String, title::String, time_range::Union{Nothing, Tuple{Float64, Float64}} = nothing)
    # Read the file line by line
    lines = readlines(file_path)

    # Parse the data into a matrix
    data = [parse.(Float64, split(line, " ")) for line in lines]

    # Convert the matrix of parsed data into separate columns
    time = [row[1] for row in data]    # First column is time
    data1 = [row[2] for row in data]  # Second column
    data2 = [row[3] for row in data]  # Third column
    data3 = [row[4] for row in data]  # Fourth column
    data4 = [row[5] for row in data]  # Fifth column

    # If time_range is provided, filter the data
    if time_range !== nothing
        # Extract the start and end of the time range
        start_time, end_time = time_range

        # Filter the data based on the time range
        indices = findall(x -> start_time <= x <= end_time, time)
        time = time[indices]
        data1 = data1[indices]
        data2 = data2[indices]
        data3 = data3[indices]
        data4 = data4[indices]
    end

    # Create the plot
    p = plot(time, data1, label="Data 1", xlabel="Time", ylabel="Values", lw=2)
    plot!(p, time, data2, label="Data 2", lw=2)
    plot!(p, time, data3, label="Data 3", lw=2)
    plot!(p, time, data4, label="Data 4", lw=2)

    # Add title
    title!(p, title)

    savefig(joinpath(@__DIR__, "..", "data", "Images", title * ".png"))  # Save as PNG
end

function subsample_csv(input_file::String, subsampling_factor::Int, output_file::String)
    if subsampling_factor < 1
        error("Subsampling factor must be a positive integer.")
    end

    # Read the input CSV file into a DataFrame
    df = CSV.read(input_file, DataFrame)

    # Subsample the DataFrame
    subsampled_df = df[1:subsampling_factor:end, :]

    # Write the subsampled data to the output file
    CSV.write(output_file, subsampled_df)

    println("Subsampled data written to: $output_file")
end