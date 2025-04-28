import matplotlib.pyplot as plt
import numpy as np

# this is file is only valid when the hybrid has been changed such that it moves in the same direction as the real robot
# -> cfr line 278 in RobotSimulator.jl

verbose = False
#----------------------------------------------------------------------------
#                       Code PARAMETERS
#----------------------------------------------------------------------------
File = "Position"
y_units = "[rad]"

#----------------------------------------------------------------------------
#                       Folder DETAILS
#----------------------------------------------------------------------------
columns_names = ["", "Left Hip", "Right Hip", "Left Knee", "Right Knee"]
columns_hybrid = [2, 1, 4, 3]

number_of_files = 3

Simulation_file_p = "../data/simulation/" + "voltage-input" + "/" + File + ".txt"
if(File == "Torque"):
    Robot_file = "../data/Robot_200Hz/Outputs/" + File + ".txt"
else:
    Robot_file = "../data/Robot_200Hz/Inputs/" + File + ".txt"
WP = "../WalkingPatterns/ZMP.csv"

save_folder = "DigitalTwin/voltage-input"

#----------------------------------------------------------------------------
#                       Load DATA
#----------------------------------------------------------------------------

data_simu_p = np.loadtxt(Simulation_file_p)
data_robot = np.loadtxt(Robot_file)
data_WP = np.loadtxt(WP, delimiter=',', skiprows=1)

#----------------------------------------------------------------------------
#                       Make PLOTS
#----------------------------------------------------------------------------

for i in range(len(columns_names)-1):
    plt.figure()

    # Plot parameters
    plt.xlabel('time [s]')
    plt.ylabel(File + " " + y_units)
    #plt.title(columns_names[i+1] + " " + File)
    plt.xlim(0, 5) # 5s plots

    # Plot data
    if(File != "Velocity"):
        plt.plot(data_robot[:,0], data_robot[:,i+1], label = 'Robot', color='black', linewidth=1.5)
    plt.axvline(x=2, color='purple', linewidth=1)
    plt.plot(data_simu_p[:,0], data_simu_p[:,i+1], label = 'Simulation prismatic', color='tab:green', linestyle='-', linewidth=1.5)
    if(File == "Position"):
        plt.plot(data_WP[:,0], data_WP[:,i+1], label = 'WalkingPattern (q_ref)', color='tab:red', linestyle=':', linewidth=1.5)
    if(File == "Velocity"):
        plt.plot(data_robot[:,0], data_robot[:,i+1], label = 'Robot', color='black', linewidth=1.5)
    plt.legend()
    if(verbose):
        plt.show()
    else:
        filename_save = save_folder + File + " " + columns_names[i+1] + '.pdf'
        plt.savefig(filename_save)