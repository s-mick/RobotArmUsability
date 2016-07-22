# -*- coding: utf-8 -*-
"""
Basic functions to generate and save a shuffled target order based on an available targets set.
Also has some utility functions to create a target set as a subset of a sphere
"""

import numpy as np
import os.path, sys
from gripperarm import GripperArm

sys.path=sys.path+['', 'C:\\Anaconda2\\python27.zip', 'C:\\Anaconda2\\DLLs', 'C:\\Anaconda2\\lib', 'C:\\Anaconda2\\lib\\plat-win', 'C:\\Anaconda2\\lib\\lib-tk', 'C:\\Anaconda2', 'c:\\anaconda2\\lib\\site-packages\\sphinx-1.3.5-py2.7.egg', 'c:\\anaconda2\\lib\\site-packages\\setuptools-20.3-py2.7.egg', 'C:\\Anaconda2\\lib\\site-packages', 'C:\\Anaconda2\\lib\\site-packages\\win32', 'C:\\Anaconda2\\lib\\site-packages\\win32\\lib', 'C:\\Anaconda2\\lib\\site-packages\\Pythonwin']

# Where to save output data
root = 'C:\Users\hybrid.NEUROCOG\Documents\coAdaptMyoelec\GripperUsability_exp1\data'
outputFile = os.path.join(root, 'targetOrder.npz')


def funcTargetOrder(nTrials = 15):
    """ Generates a nTrials long target order from the available targets set, looping over it if it is shorter than nTrials
    The generated target order and the targets' coordinates are saved on the disk as a .npz archive
    :param int nTrials: Length of the target order to generate
    """
    nTargets = targets.shape[0]
    order = - np.ones(0)
    
    while order.shape[0] < nTrials - nTargets:
        block = np.random.permutation(np.arange(nTargets) )
        order = np.append(order, block, 0)
    
    block = np.random.permutation(np.arange(nTargets) )[:( (nTrials - 1) % nTargets + 1)]
    order = np.append(order, block, axis = 0)
    
    targetOrder = [targets, order]
    np.savez(outputFile, targetOrder = targetOrder)

def readTargetOrderFile():
    """Reads a file produced by funcTargetOrder, prints each target's coordinates and makes the robot move to it in the corresponding order
    Very useful when the targets are chosen and must be placed on the physical setup:
    The robot points at the expected position, and the experimenter just has to place the actual target items
    """
    data = np.load(outputFile)
    targetOrder = data['targetOrder']
    print(targetOrder)
    
    arm.goto_target(init_pos, 1.5, wait = True)
    raw_input()
    for t in targetOrder[0]:
        print(t)
        arm.goto_target(t, 1.5)
        raw_input()
        arm.goto_target(init_pos, 1.5, wait = True)
    
    data.close()
    
def getTargetsOnSphere(center, radius, x, y, n = 1000):
    """Looks for points at the intersection of a sphere and a vertical line
    :param array center: 3-D coordinates of the center of the sphere
    :param float radius: radius of the sphere
    :param float x: X coordinate of the vertical line in the robot space
    :param float y: Y coordinate of the vertical line in the robot space
    :param int n: Optional: Number of points on the line, possibly an arbitrarily big number, but should match the desired accuracy.
    :return: Array of points near the intersection, if it exists
    """
    rz = np.linspace(0, 0.70, n)
    rx = np.repeat(x, n)
    ry = np.repeat(y, n)
    
    r = np.zeros((n, 3))
    r[:, 0] = rx
    r[:, 1] = ry
    r[:, 2] = rz
    
    res = []
    for M in r:
        if abs(np.linalg.norm(M - center) - radius) < 0.3 / n:
            res.append(M)
    
    return np.array(res)

if __name__ == '__main__':
    
    # Positions of the available targets
    targets = np.array([[0.2, 0.01, 0.284],
                        [0.29, -0.055, 0.302],
                        [0.21, -0.15, 0.372],
                        [0.23, -0.265, 0.309],
                        [0.065, -0.26, 0.169] ])
    
    # Position of the initial position
    init_pos = np.array([0.18, -0.14, 0.205])
    
    # Generate a target order
#    funcTargetOrder(20)
    
    # Connect to the robot
    arm = GripperArm('noclamp')
#    arm.goto_target(init_pos, 1.5, wait = True)
#    raw_input()
#    arm.goto_target([0.32, -0.06, 0.27], 1.5, wait = True)
    # Read the generated target order and make the robot move to each target

    readTargetOrderFile()

    arm.shutdown() # Close the robot

#    print(getTargetsOnSphere(np.array([0.18, -0.14, 0.205]), 0.17, 0.23, -0.265) )