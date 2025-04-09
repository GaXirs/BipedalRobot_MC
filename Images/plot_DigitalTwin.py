import matplotlib.pyplot as plt
import numpy as np

verbose = False
#----------------------------------------------------------------------------
#                       Code PARAMETERS
#----------------------------------------------------------------------------
File = "Position"
URDF = "Prismatic" # "Hybrid"
columns_names = data = ["", "Left Hip", "Right Hip", "Left Knee", "Right Knee"]
y_units = "[rad]"

#----------------------------------------------------------------------------
#                       Folder DETAILS
#----------------------------------------------------------------------------
number_of_files = 3

Simulation_file = "../data/simulation/" + URDF + "/" + File + ".txt"
if(File == "Torque"):
    Robot_file = "../data/Robot_50Hz/Outputs/" + File + ".txt"
else:
    Robot_file = "../data/Robot_50Hz/Inputs/" + File + ".txt"
WP = "../WalkingPatterns/ZMP.csv"

save_folder = "DigitalTwin/" + URDF + "/"

#----------------------------------------------------------------------------
#                       Load DATA
#----------------------------------------------------------------------------

data_simu = np.loadtxt(Simulation_file)
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
    plt.title(columns_names[i+1] + " " + File)
    plt.xlim(0, 5) # 5s plots

    # Plot data
    plt.plot(data_simu[:,0], data_simu[:,i+1], label = 'Simulation')
    plt.plot(data_robot[:,0], data_robot[:,i+1], label = 'Robot')
    if(File == "Position"):
        plt.plot(data_WP[:,0], data_WP[:,i+1], label = 'WalkingPattern (q_ref)')
    plt.legend()
    if(verbose):
        plt.show()
    else:
        filename_save = save_folder + File + " " + columns_names[i+1] + '.pdf'
        plt.savefig(filename_save)