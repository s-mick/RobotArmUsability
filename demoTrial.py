# -*- coding: utf-8 -*-
"""
Script faking a trial, allowing the experimenter to move the robot by hand and demonstrate to the subject how a trial should take place.
"""

import numpy as np
import sys
from gripperarm import GripperArm
from winsound import Beep

sys.path=sys.path+['', 'C:\\Anaconda2\\python27.zip', 'C:\\Anaconda2\\DLLs', 'C:\\Anaconda2\\lib', 'C:\\Anaconda2\\lib\\plat-win', 'C:\\Anaconda2\\lib\\lib-tk', 'C:\\Anaconda2', 'c:\\anaconda2\\lib\\site-packages\\sphinx-1.3.5-py2.7.egg', 'c:\\anaconda2\\lib\\site-packages\\setuptools-20.3-py2.7.egg', 'C:\\Anaconda2\\lib\\site-packages', 'C:\\Anaconda2\\lib\\site-packages\\win32', 'C:\\Anaconda2\\lib\\site-packages\\win32\\lib', 'C:\\Anaconda2\\lib\\site-packages\\Pythonwin']


if __name__ == '__main__':
    # Positions of the available targets
    target = np.array([0.2, 0.01, 0.284])                  
    init_pos = np.array([0.18, -0.14, 0.205])
    
    # Instantiate the robot arm and make it move to the starting position
    arm = GripperArm('noclamp')
    arm.goto_target(init_pos, 1.5, wait = True)
    
    # Make the robot compliant when pressing Enter, so it can be freely manipulated
    raw_input()
    arm.robot.compliant = True
    
    continueLoop = True
    exitedStart = False
    onTarget = False
    counter = 0
    
    while continueLoop:
        pos = arm.get_endpoint_position()
        
        if (not exitedStart) and (np.linalg.norm(pos - init_pos) > 0.02):
            Beep(2400, 150) # Beep when exiting the starting zone for the first time
            exitedStart = True
            
        if np.linalg.norm(pos - target) < 0.02:
            if onTarget:
                counter += 1
                if counter > 6000:
                    Beep(600, 700) # Beep when validating the target
                    continueLoop = False
            else:
                Beep(1200, 300) # Beep when entering the target zone
                onTarget = True
        else:
            onTarget = False
            counter = 0

    arm.shutdown()