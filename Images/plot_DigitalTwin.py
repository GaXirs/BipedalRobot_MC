import matplotlib.pyplot as plt
import numpy as np

verbose = False
#----------------------------------------------------------------------------
#                       Code PARAMETERS
#----------------------------------------------------------------------------
File = "Voltage"
y_units = "[V]"

#----------------------------------------------------------------------------
#                       Folder DETAILS
#----------------------------------------------------------------------------
columns_names = data = ["", "Left Hip", "Right Hip", "Left Knee", "Right Knee"]

number_of_files = 4

Simulation_file_p = "../data/simulation/" + "Prismatic" + "/" + File + ".txt"
Simulation_file_h = "../data/simulation/" + "Hybrid" + "/" + File + ".txt"
if(File == "Torque"):
    Robot_file = "../data/Robot_200Hz/Outputs/" + File + ".txt"
else:
    Robot_file = "../data/Robot_200Hz/Inputs/" + File + ".txt"
WP = "../WalkingPatterns/ZMP.csv"

save_folder = "DigitalTwin/"

#----------------------------------------------------------------------------
#                       Load DATA
#----------------------------------------------------------------------------

data_simu_p = np.loadtxt(Simulation_file_p)
data_simu_h = np.loadtxt(Simulation_file_h)
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
    plt.plot(data_simu_p[:,0], data_simu_p[:,i+1], label = 'Simulation prismatic')
    plt.plot(data_simu_h[:,0], data_simu_h[:,i+1], label = 'Simulation hybrid', color='purple', linestyle = ":")
    plt.plot(data_robot[:,0], data_robot[:,i+1], label = 'Robot')
    if(File == "Position"):
        plt.plot(data_WP[:,0], data_WP[:,i+1], label = 'WalkingPattern (q_ref)')
    plt.legend()
    if(verbose):
        plt.show()
    else:
        filename_save = save_folder + File + " " + columns_names[i+1] + '.pdf'
        plt.savefig(filename_save)