# -*- coding: utf-8 -*-

"""
For one given subject, plots the trajectories of the endpoint
"""

# Scientific calculus utilities
import numpy as np                  # Basic array structures
import matplotlib.pyplot as plt     # Plotting tools

import os.path



def readFile(inputFile, container, trialIndex):
    """Reads metrics data from the given input file
    :param string inputFile: Path to the input file to read
    :param list container: List where to append the read and formated data.
    :returns: The same container with appended metrics data. If the container was empty at first, metrics names are appended as the first element.
    """
    # Load from file
    data = np.load(inputFile)
    trajData = data['trajData']
    data.close()

    # Reformat into a np.array
    array = np.zeros( (3, trajData.shape[0]) )
    for i, n in enumerate(trajData.dtype.names):
        array[i, :] = trajData[n]
    
    container[trialIndex] = array

def browseForFiles(mode, subfolder):
    """Browses the data folder looking for metrics data files, and generates an array to format and stock retrieved data
    :param string mode: Control mode identifier
    :param string subfolder: Name of the subject folder
    :returns: Tuple comprising the list of lists of 3.nTimesteps shape arrays
    """
    # Look for every file named '*Traj.npz' in the current folder and all its subfolders, and put their data in a list
    container = [0] * nTrials
    for root, dirs, files in os.walk('.\\set1\\{}\\{}\\'.format(mode, subfolder) ): 
        for name in files:
            if name[-8:] == 'Traj.npz':
                readFile(os.path.join(root, name), container, int(name[5:-8]) - 1)

    return container


if __name__ == '__main__' :
    
    groupNames = ['Pos', 'Vel', 'Vel2']
    nGroups = len(groupNames)
    # Load and format raw force data 
    nTrials = 20
    
    folder = 'Giulia13-07-16_12h'
    dPos = browseForFiles('pos', folder + '30')
    dVel = browseForFiles('vel', folder + '17')
    dVel2 = browseForFiles('vel2', folder + '41')
    dAll = [dPos, dVel, dVel2]
    print('Trajectory data loaded')
    
    # Read data from targetOrder
    targetData = np.load(os.path.join('.\\set1', 'targetOrder.npz' ) )
    targetOrder = targetData['targetOrder'][1]
    targetPos = targetData['targetOrder'][0]
    init_pos = np.array([0.18, -0.14, 0.205])
    
    # Pre-calculate the unit vectors representing the ideal movement direction from the starting point towards the target
    targetVect = np.zeros( (len(targetPos), 2) )
    for i, target in enumerate(targetPos):
        v = (target - init_pos)[:2]
        targetVect[i, :] = v / np.linalg.norm(v)
    
    figs = [0] * len(targetPos)
    
    for i, indt in enumerate(targetOrder):
        it = int(indt)
        target = targetPos[it, :] - init_pos
        uvect = targetVect[it, :]
        tproj = np.dot(target[:2], uvect)
        
        for g, gname in enumerate(groupNames):
            if figs[it] == 0: # Create one figure per target only
                fig, axes = plt.subplots(2, 2, sharex = True, sharey = True)
                fig.suptitle('Trajectories in the vertical plane joining the starting point and the target', fontsize = 14)
                axes[0][0].axis('off')
                figs[it] = fig
            else:
                fig = figs[it]
                
            ax = fig.axes[g + 1] # Get the corresponding subplot
            
            # Project and plot the trajectories on a plane containing the starting point, the target and the z axis
            traj = np.array(dAll[g][i]).T - init_pos
            proj = np.sum(traj[:, :2] * uvect, axis = 1)
            ax.plot(proj, traj[:, 2], 'b')
            
            # Plot target and starting zone circles
            circ_start = plt.Circle([0, 0], 0.02, color = 'g', fill = False, lw = 2)
            circ_targ = plt.Circle([tproj, targetPos[it, 2] - init_pos[2] ], 0.02, color = 'r', fill = False, lw = 2)
            ax.add_patch(circ_start)
            ax.add_patch(circ_targ)
            ax.set_title(gname)
            ax.set_xlabel('Horizontal distance to target')
            ax.set_ylabel('Z axis')
            if i < 5:
                ax.legend(['Trajectory', 'Starting zone', 'Target'], bbox_to_anchor=(0, 1), loc = 2, borderaxespad = 0.)
            
            ax.axis('equal')
            ax.grid(True, which = 'both')            
    
    plt.show()