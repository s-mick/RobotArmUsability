# !/usr/bin/env python2
#  -*- coding: utf-8 -*-
"""
Main framework for the usability assessment study on three control modes operating the robotic arm
PsychoPy works only as a timer, event handler and display utility.
Indeed, it's clearly unnecessary to run this mega machine for such basic functionalities, but I guess I was lazy...
"""

import os  # handy system and path functions
import sys # to get file system encoding

sys.path = sys.path + ['', 'C:\\Anaconda2\\python27.zip', 'C:\\Anaconda2\\DLLs', 'C:\\Anaconda2\\lib', 'C:\\Anaconda2\\lib\\plat-win', 'C:\\Anaconda2\\lib\\lib-tk', 'C:\\Anaconda2', 'c:\\anaconda2\\lib\\site-packages\\sphinx-1.3.5-py2.7.egg', 'c:\\anaconda2\\lib\\site-packages\\setuptools-20.3-py2.7.egg', 'C:\\Anaconda2\\lib\\site-packages', 'C:\\Anaconda2\\lib\\site-packages\\win32', 'C:\\Anaconda2\\lib\\site-packages\\win32\\lib', 'C:\\Anaconda2\\lib\\site-packages\\Pythonwin']

from psychopy import visual, core, event#, sound, gui, locale_setup, logging
from psychopy.constants import NOT_STARTED, STARTED, FINISHED
#from __future__ import division  # so that 1/3=0.333 instead of 1/3=0
import numpy as np
from matplotlib import pyplot as plt
#from numpy import sin, cos, tan, log, log10, pi, average, sqrt, std, deg2rad, rad2deg, linspace, asarray
#from numpy.random import random, randint, normal, shuffle

from Tkinter import Tk
import threading, time
from winsound import Beep

from PyDAQmx import Task
from PyDAQmx.DAQmxFunctions import int32, byref
from PyDAQmx.DAQmxConstants import DAQmx_Val_RSE
from PyDAQmx.DAQmxConstants import DAQmx_Val_Volts, DAQmx_Val_Rising
from PyDAQmx.DAQmxConstants import DAQmx_Val_Acquired_Into_Buffer
from PyDAQmx.DAQmxConstants import DAQmx_Val_ContSamps, DAQmx_Val_GroupByChannel

from gripperarm import GripperArm


class Configuration(object):
    """Shallow class acting as a container for configuration info
    """
    def __init__(self):
        pass

class GoalDriving():
    """Class gathering the calculus methods that are used to compute the arm's goal
    """
    def __init__(self, config):
#        self.linear_offset = config.linear_offset # Offset corresponding to the weight of the handle on the force transducer
        self.angular_offset = config.angular_offset # Offset corresponding to the alignment of the transducer along its Z axis
        self.gain = config.gain # Conversion gain from force to position. Should be a 3x1 array rather than a scalar!!
        
        self.init_pos = config.init_pos
        
        self.mode = config.mode # Control mode name. Should be 'pos', 'vel' or 'vel2'. Default is 'pos'
        if self.mode == 'pos':
            self.calcGoal = self.calcGoal_Pos
        elif self.mode == 'vel':
            self.calcGoal = self.calcGoal_Vel
            self.vel_threshold = config.vel_threshold
        elif self.mode == 'vel2':
            self.calcGoal = self.calcGoal_Vel2
            self.vel_threshold = config.vel_threshold
        else:
            self.calcGoal = self.calcGoal_Pos
            
        self.goal_trajectory = [config.init_pos]
    
    def pol2cart(self, rho, phi):
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return x, y
        
    def cart2pol(self, x, y):
        rho = np.sqrt(x**2 + y**2)
        phi = np.arctan2(y, x)
        return rho, phi
    
    def calcGoal_Pos(self, forces, dt = None):
        """
        Determines the 3-D position of the current instantaneous goal for the robot arm's endpoint, based on the force measurements, for a position control mode
        :param array forces: Array of raw data retrieved from the three force channel, should be a 3.nSamples array
        :param float dt: Useless in position control but acts to respect the 3-argument format of the general calcGoal method
        :returns: Three-element tuple: goal, the 3-D coordinates of the point to reach; rectified force data; realigned raw force data
        """        
        # Compute means on the batch's samples from each channel. Should be a 3.1 array        
        val = np.mean(forces, axis = 1) - self.linear_offset
        
        # Compute the goal's position in the robot's frame
        (rho, phi) = self.cart2pol(val[0], val[1])
        (x, y) = self.pol2cart(rho, phi + self.angular_offset) # Counterbalance the angular offset of the force transducer
        goal = self.init_pos + self.gain * np.array([val[2], x, y])
        
        self.goal_trajectory.append(goal)
        return goal, self.gain * np.array([val[2], x, y] ), np.array([val[2], x, y])
        
    def calcGoal_Vel(self, forces, dt):
        """
        Determines the 3-D position of the current instantaneous goal for the robot arm's endpoint, based on the force measurements, for a velocity control mode
        :param array forces: Array of raw data retrieved from the three force channel, should be a 3.nSamples array
        :param float dt: time elapsed since the last control update
        :returns: Three-element tuple: goal, the 3-D coordinates of the point to reach; rectified force data; realigned raw force data
        """
        val = np.mean(forces, axis = 1) - self.linear_offset
        
        # Compute the goal's velocity in the robot's frame
        (rho, phi) = self.cart2pol(val[0], val[1])
        (x, y) = self.pol2cart(rho, phi + self.angular_offset)
        vp = self.gain * np.array([val[2], x, y])
        
        if np.linalg.norm(vp) < self.vel_threshold: # Ignore low velocities [m/s]
            vp = np.zeros(3)
        goal = np.array(self.goal_trajectory[-1]) + vp * dt
        
        self.goal_trajectory.append(goal)
        return goal, vp , np.array([val[2], x, y])
        
    def calcGoal_Vel2(self, forces, dt):
        """
        Determines the 3-D position of the current instantaneous goal for the robot arm's endpoint, based on the force measurements, for a quadratic velocity control mode
        :param array forces: Array of raw data retrieved from the three force channel, should be a 3.nSamples array
        :param float dt: time elapsed since the last control update
        :returns: Three-element tuple: goal, the 3-D coordinates of the point to reach; rectified force data; realigned raw force data
        """
        val = np.mean(forces, axis = 1) - self.linear_offset
        
        # Compute the goal's velocity in the robot's frame
        (rho, phi) = self.cart2pol(val[0], val[1])
        (x, y) = self.pol2cart(rho, phi + self.angular_offset)
        vect = np.array([val[2], x, y])
        vp = self.gain * vect * np.linalg.norm(vect) # Compute quadratic magnitude without changing the sign
        
        if np.linalg.norm(vp) < self.vel_threshold: # Ignore low velocities [m/s]
            vp = np.zeros(3)
        goal = np.array(self.goal_trajectory[-1]) + vp * dt
        
        self.goal_trajectory.append(goal)
        return goal, vp, np.array([val[2], x, y])
    
    def calcOffset(self, forces):
        """
        Determines the offset that will be applied to the force measurements when computing the goal
        :param array forces: Array of raw data retrieved from the three force channel, should be a 3.nSamples array
        """
        self.linear_offset = np.mean(forces, axis = 1)
        print(self.linear_offset)
        
        
class InitGoalDriving(object):
    """Class gathering data about targets, acquisition process and data saving
    """
    def __init__(self, config, calc):
        self.config = config
        
        self.dirData = config.dirData
        self.subjectName = config.subjectName
        # Path to the subject's directory, with name and timestamp
        fullpath = os.path.join(config.dirData, config.mode, config.subjectName + time.strftime('%d-%m-%y_%Hh%M') )
        if not os.path.isdir(fullpath):
            os.mkdir(fullpath)
        self.dirSubject = fullpath
        
        self.init_pos = config.init_pos # Starting position of the arm's endpoint
        self.init_pos_radius = config.init_pos_radius # Radius of the starting zone, centered on the starting position
        self.target_radius = config.target_radius # Radius of the target dwelling zone
        
        self.trialDelay = config.trialDelay
        self.dwellingTime = config.dwellingTime
        
        self.frequency = config.frequency
        self.nSamplesPerUpdate = config.nSamplesPerUpdate
        self.nSamplesPerBatch = config.nSamplesPerBatch
        self.instruction_duration = config.instruction_duration
        
        self.configChannelAcqui()
        self.configInterface()
        self.initialisation()
   
    def configChannelAcqui(self):
        """Configures the channel from which to retrieve data
        """
        # Addresses of the channels to acquire
        self.numCarteAcqui = 2
        self.basenameChannel = 'Dev{0}/ai'.format(self.numCarteAcqui)
        self.channelListAcqui = self.basenameChannel + '0:23'
        self.nChannels = 24 # Acquire ALL 24 channels

        # Voltage boundaries for acquisition
        self.limitInf = -10
        self.limitSup = 10
        
        self.nForces = self.config.nForcesRecorded if self.config.nForcesRecorded < 4 else 3
        self.channelSelect = range(0, self.nForces) if self.nForces else []
            
    def configInterface(self):
        """Configures the interface, i.e. the monitor that is used as display, and the targets' positions
        """
        # Load conditions from targetOrder
        chosenFile = 'targetOrder.npz'
        
        # Read data from targetOrder
        data = np.load(os.path.join(self.dirData, '{0}'.format(chosenFile) ) )
        self.targetOrder = data['targetOrder'][1]
        self.nTargets = len(set(self.targetOrder) ) # Total number of targets
        self.targets = data['targetOrder'][0]
        
        # Create a dictionary of info about the experiment
        self.expInfo = {u'participant': self.subjectName}
        self.expInfo['date'] = time.strftime('%d-%m-%y_%Hh%M')
                
        # 'Experiment ended' flag if 'escape' is received or another ending condition occurs
        self.endExpNow = False
        
        ##############################################################################
        # Start Code - component code to be run before the window creation #
        ##############################################################################
        
        # Window configuration        
        self.win = visual.Window(size=(self.config.screenWidth, self.config.screenHeight), fullscr=True, allowGUI=False,
            monitor='monitor_{0}x{1}'.format(self.config.screenWidth,self.config.screenHeight), useFBO=True, color = (-1, -1, -1), colorSpace = 'rgb')
        
        # Get display frequency if available
        # If not, assume 60 Hz
        self.expInfo['framerate'] = self.win.getActualFrameRate()
        if self.expInfo['framerate'] != None:
            self.durationFrame = 1.0 / round(self.expInfo['framerate'])
        else:
            self.durationFrame = 1 / 60.0
           
    def initialisation(self):
        """
        Initialisation of the different routines' components
        """
        
        # Instruction
        self.instructionClock = core.Clock()
        self.instructionText = visual.TextStim(win = self.win, name = 'instructionText',
            text = 'L\'experience va bientot commencer. Ne touchez pas la poignee. Calibration en cours...', font = 'Arial',
            height = 0.1, color = 'green', depth = -1.0)
        
        # Relax
        self.relaxClock = core.Clock()
        self.relaxText = visual.TextStim(win = self.win, name = 'relaxText',
            text = 'None', font = 'Arial', height = 0.1, color = 'green')
        
        # Reach target
        self.reachClock = core.Clock()
        self.dwellTimer = core.CountdownTimer()
        
        # Save
        self.saveClock = core.Clock()
        

class CallbackTaskSynchronous(Task):
    """Custom callback task used for data acquisition from the force transducer
    """
    def __init__(self, channelName='Dev2/ai0:23', dataLen = 40, frequency = 2000, nChannels = 24):
        Task.__init__(self)
        self.dataLen = dataLen
        self.frequency = frequency
        self.nChannels = nChannels
        self._data = np.zeros( (self.nChannels, self.dataLen) )
        self.read = int32()
        self.CreateAIVoltageChan(channelName, "", DAQmx_Val_RSE, -10.0, 10.0, DAQmx_Val_Volts, None)
        self.CfgSampClkTiming("", self.frequency, DAQmx_Val_Rising, DAQmx_Val_ContSamps, self.dataLen)
        self.AutoRegisterEveryNSamplesEvent(DAQmx_Val_Acquired_Into_Buffer, self.dataLen, 0)
        self.AutoRegisterDoneEvent(0)
        self._data_lock = threading.Lock()
        self._newdata_event = threading.Event()
    
    def EveryNCallback(self):
        with self._data_lock:
            self.ReadAnalogF64(self.dataLen, 10.0, DAQmx_Val_GroupByChannel, self._data, self.nChannels * self.dataLen, byref(self.read), None)
            self._newdata_event.set()
        return 0 # The function should return an integer
    
    def DoneCallback(self, status):
        print('Status: {}'.format(status.value) )
        return 0 # The function should return an integer
    
    def get_data(self, blocking=True, timeout = None):
        if blocking:
            if not self._newdata_event.wait(timeout):
                raise ValueError("Timeout waiting for data from device")
        with self._data_lock:
            self._newdata_event.clear()
            return self._data.copy()

def funcGoalDriving(config):       
    calc = GoalDriving(config)
    paramInit = InitGoalDriving(config, calc)
    task=CallbackTaskSynchronous(channelName = paramInit.channelListAcqui, dataLen = config.nSamplesPerUpdate, frequency = paramInit.frequency, nChannels = paramInit.nChannels)
    arm = GripperArm(version = 'noclamp')
    
    # Create a timer to get the time remaining for each routine
    routineTimer = core.CountdownTimer()
    
    """
    Execution routine : <<< INSTRUCTION >>>
    """
    #########################################################
    #------Prepare to start Routine "Instruction"-------#
    #########################################################
    t = 0
    paramInit.instructionClock.reset()
    frameN = -1 # Reset the frame counter
    PressEnter = event.BuilderKeyResponse()
    PressEnter.status = NOT_STARTED # Initial status of the event
    
    # Gather the components
    instructionComponents = []
    instructionComponents.append(PressEnter)
    instructionComponents.append(paramInit.instructionText)
    for thisComponent in instructionComponents:
        if hasattr(thisComponent, 'status'):
            thisComponent.status = NOT_STARTED
    
    task.StartTask()
    allAcqui = {}
    deb = 0
    
    ######################################################
    #-------Start Routine "Instruction"-------
    ######################################################
    continueRoutine = True
    while continueRoutine:
        # Get the current time
        t = paramInit.instructionClock.getTime()
        frameN = frameN + 1
        
        # Acquire a bunch of raw data
        oneAcqui = task.get_data()
        if deb == 0:
            allAcqui = oneAcqui
            deb = 1
        else:
            allAcqui = np.append(allAcqui, oneAcqui, axis = 1)
        
        # Update event
        if t >= 0.0 and PressEnter.status == NOT_STARTED:
            # Save the starting time and the frame number
            PressEnter.tStart = t
            PressEnter.frameNStart = frameN
            PressEnter.status = STARTED
            # Configure the system interruptions
            paramInit.win.callOnFlip(PressEnter.clock.reset)  # t=0 on the next frame update
            event.clearEvents(eventType='keyboard')
        if t >= paramInit.instruction_duration + 1.0 and PressEnter.status == STARTED:
            theseKeys = event.getKeys(keyList=['return']) # Get the pressed keys
            
            # Check if exit
            if 'escape' in theseKeys:
                paramInit.endExpNow = True
            if len(theseKeys) > 0:  # At least one key was pressed
                PressEnter.keys = theseKeys[-1]  # Only remember the last one
                PressEnter.rt = PressEnter.clock.getTime()
                # Routine end
                continueRoutine = False
        
        # Update text
        if t >= 0.0 and paramInit.instructionText.status == NOT_STARTED:
            # Save the starting time and the frame number
            paramInit.instructionText.tStart = t
            paramInit.instructionText.frameNStart = frameN
            paramInit.instructionText.setAutoDraw(True) #tracé à chaque frame
        
        # Check if all components are started
        if not continueRoutine:  # If one called the end of the routine
            break
        continueRoutine = False  # This attribute will come back to True if at least one component is still active
        for thisComponent in instructionComponents:
            if hasattr(thisComponent, 'status') and thisComponent.status != FINISHED:
                continueRoutine = True
                break  # At least one component is still active
        
        # Check for the 'escape' key
        if paramInit.endExpNow or event.getKeys(keyList=['escape']):
            core.quit()
        
        # Update frame
        if continueRoutine:
            paramInit.win.flip()
    
    ######################################################
    #-------Ending Routine "Instruction"-------
    ######################################################
    task.StopTask()
    
    for thisComponent in instructionComponents:
        if hasattr(thisComponent, 'setAutoDraw'):
            thisComponent.setAutoDraw(False)
    
    # Display some curves to check the envelope of the raw data
#    fig = plt.figure()
#    cols = ['r', 'g', 'b']
#    lab = ['fX', 'fY', 'fZ']
#    for c in range(3):
#        pax = fig.add_subplot(3, 1, c + 1)
#        pax.plot(allAcqui[c, :], cols[c], label = lab[c])
#        plt.ylabel('Measured efforts (V)')
#        plt.legend(bbox_to_anchor=(1, 1), loc = 1, borderaxespad = 0.)
    
    # Compute the linear offset to apply to the measured forces
    wdata = allAcqui[:3, (- paramInit.instruction_duration * paramInit.frequency):] # Pick the samples retrieved during the relax_duration seconds
    calc.calcOffset(wdata)
    
    print('Instruction routine ended\n')
    
    ###################################################################################################################
    # Declare data types and arrays that will be used when saving measurements
    forceDNames = ['Fx', 'Fy', 'Fz']
    forceDType = {}
    forceDType['names'] = forceDNames
    forceDType['formats'] = ['f8'] * len(forceDNames)
    
    trajDNames = ['Px', 'Py', 'Pz']
    trajDType = {}
    trajDType['names'] = trajDNames
    trajDType['formats'] = ['f8'] * len(trajDNames)
    
    metricDNames = ['RT','VT', 'MOR', 'PO', 'S'] # Reach Time, Validation Time, Maximum Overshoot Ratio, Path Optimality, Success (0/1)
    metricDType = {}
    metricDType['names'] = metricDNames
    metricDType['formats'] = ['f8'] * len(metricDNames)
    
    metricData = np.zeros(len(paramInit.targetOrder), dtype = metricDType)
    
    # Loop over the targets
    for thisTrial, thisTargetIndex in enumerate(paramInit.targetOrder):
        thisTarget = paramInit.targets[thisTargetIndex]
        ###################################################################################################################        
        """
        Execution routine : <<< RELAX >>>
        """
        ######################################################
        #------Prepare to start Routine "Relax"-------#
        ######################################################
        # Reset the robot's whole position, and not only its endpoint's position
        arm.goto_target(config.init_pos, 1, initial_pos = [0] * len(arm.chain.links), wait = True)
        calc.goal_trajectory = [config.init_pos] # Reset the goal trajectory before going on with another trial
        arm_trajectory = [config.init_pos] # Container for actual arm endpoint trajectory
        
        t = 0
        paramInit.relaxClock.reset()
        paramInit.relaxText.text = 'Quelques secondes de repos.\n\nProchaine cible : {}\nEssai {}'.format(chr(65 + int(thisTargetIndex) ), thisTrial + 1)
        frameN = -1
        RelaxPressEnter = event.BuilderKeyResponse()
        
        relaxComponents = []
        relaxComponents.append(paramInit.relaxText)
        relaxComponents.append(RelaxPressEnter)
        for thisComponent in relaxComponents:
            if hasattr(thisComponent, 'status'):
                thisComponent.status = NOT_STARTED
        
        ######################################################
        #-------Start Routine "Relax"-------
        ######################################################
        continueRoutine = True
        while continueRoutine:
            t = paramInit.relaxClock.getTime()
            frameN = frameN + 1
            
            # Update text
            if t >= 0.0 and paramInit.relaxText.status == NOT_STARTED:
                paramInit.relaxText.tStart = t
                paramInit.relaxText.frameNStart = frameN
                paramInit.relaxText.setAutoDraw(True)
            
            # Update event
            if t >= 0.0 and RelaxPressEnter.status == NOT_STARTED:
                RelaxPressEnter.tStart = t
                RelaxPressEnter.frameNStart = frameN
                RelaxPressEnter.status = STARTED
                
                paramInit.win.callOnFlip(RelaxPressEnter.clock.reset)
                event.clearEvents(eventType='keyboard')
            if (t >= 1.0) and RelaxPressEnter.status == STARTED:
                theseKeys = event.getKeys(keyList=['return'])
                
                # Check if 'escape' key was pressed
                if 'escape' in theseKeys:
                    paramInit.endExpNow = True
                if len(theseKeys) > 0:
                    RelaxPressEnter.keys = theseKeys[-1]
                    RelaxPressEnter.rt = RelaxPressEnter.clock.getTime()
                    
                    continueRoutine = False
            
            if not continueRoutine:
                break
            continueRoutine = False
            for thisComponent in relaxComponents:
                if hasattr(thisComponent, "status") and thisComponent.status != FINISHED:
                    continueRoutine = True
                    break
            
            if paramInit.endExpNow or event.getKeys(keyList=['escape']):
                core.quit()
            
            if continueRoutine:
                paramInit.win.flip()
        
        #-------Ending Routine "Relax"-------
        for thisComponent in relaxComponents:
            if hasattr(thisComponent, 'setAutoDraw'):
                thisComponent.setAutoDraw(False)

        if RelaxPressEnter.keys in ['', [], None]:  # If no key was pressed
            RelaxPressEnter.keys = None            
        
        print('Relax routine ended\n')

        ###################################################################################################################
        """
        Execution routine : <<< REACH >>>
        """
        #################################################################
        #------Prepare to start Routine "Reach"-------
        #################################################################
        paramInit.timeOnExit = None
        paramInit.timeOnTarget = None
        times = [0]
        t = 0
        paramInit.reachClock.reset()
        frameN = -1
        
        # Get ready to retrieve data from the acquisition board
        task.StartTask()
        task.get_data() # Ugly way to wash the buffer?
        allAcqui={}
        deb = 0
        
        # Date at which the goal first got out of the starting zone
        startDate = -1
        # Date at which the goal first reached the target
        reachDate = -1
        armOnTarget_bool = False
        # Date at which the target is validated
        dwellDate = -1
        
        #################################################################
        #-------Start Routine "Reach"-------
        #################################################################
        continueRoutine = True
        countdownStarted = False
        vals = [np.zeros(3)]
        forces = [np.zeros(3)]
        while continueRoutine and not (countdownStarted and routineTimer.getTime() <= 0):
            t = paramInit.reachClock.getTime()
            dt = t - times[-1]
            times.append(t)
            # Acquire a bunch of data
            oneAcqui = task.get_data()
            # Save newly acquired data with old ones
            if deb == 0:
                allAcqui = oneAcqui
                deb = 1
            else:
                allAcqui = np.append(allAcqui, oneAcqui, axis = 1)
                
            # Variable windows size: process nSamplesPerBatch samples or less, depending on the raw data array's size
            wsize = min([paramInit.nSamplesPerBatch, allAcqui.shape[1] ]) # Avoids getting stuck if not enough samples
            wdata = allAcqui[:3, -wsize:] # Pick the wsize last raw samples, which will be processed as a single batch
            
            # Compute the corresponding goal
            (goal, val, force) = calc.calcGoal(wdata, dt)
            # Make the robot move            
            move = arm.goto_target(goal, 0.1, wait = False)
            # Get the current endpoint position
            arm_pos = arm.get_endpoint_position()
            arm_trajectory.append(arm_pos)
            # If the goal wasn't reachable, save the current position as the last goal
            if move == (-1, -1):
                calc.goal_trajectory[-1] = arm_pos
                
            vals.append(val)
            forces.append(force)
            
            
            # Check if the endpoint exited the starting zone
            if np.linalg.norm(arm_pos - paramInit.init_pos) > paramInit.init_pos_radius:
                if startDate == -1:
                    startDate = t
                    routineTimer.reset(paramInit.trialDelay) # Start the movement countdown
                    countdownStarted = True
                    print('startDate: {}'.format(t) )
                    Beep(2400, 150)
            
            # Check if the target was reached
            if np.linalg.norm(arm_pos - thisTarget) < paramInit.target_radius:
                if reachDate == -1: # If it is reached for the first time
                    reachDate = t # Save the date
                    print('reachDate: {}'.format(t) )
                if armOnTarget_bool == True: # If the goal was already on target
                    if paramInit.dwellTimer.getTime() <= 0: # If the dwelling time is over
                        dwellDate = t # Save the date
                        print('dwellDate: {}'.format(t) )
                        continueRoutine = False
                else: # Rising edge
                    armOnTarget_bool = True
                    Beep(1200, 300)
                    paramInit.dwellTimer.reset(paramInit.dwellingTime) # Start dwelling countdown
            else:
                armOnTarget_bool = False
            
            if paramInit.endExpNow or event.getKeys(keyList=['escape']):
                core.quit()
    
        #################################################################
        #-------Ending Routine "Reach"-------
        #################################################################
        Beep(600, 700)
        task.StopTask()
        
        # Compute metrics
        ## Times
        if reachDate == -1: # If the goal didn't reach the target before the end of the countdown timer
            metricData['RT'][thisTrial] = paramInit.trialDelay
        else:
            metricData['RT'][thisTrial] = reachDate - startDate
            
        if dwellDate == -1: # If the goal didn't dwell long enough on the target
            metricData['VT'][thisTrial] = paramInit.trialDelay - reachDate + startDate # Can be zero
        else:
            metricData['VT'][thisTrial] = dwellDate - reachDate
            metricData['S'][thisTrial] = 1
        
        ## Trajectories
        opt_path = np.linalg.norm(thisTarget - paramInit.init_pos) # Optimal path length: straight line
        arm_traj = np.array(arm_trajectory) # Format the arm trajectory to process more efficiently. Shape: nSamples.3
        
        dists = [np.linalg.norm(pos - paramInit.init_pos) for pos in arm_traj] # Distances from the initial position to the current position at each sample
        metricData['MOR'][thisTrial] = max(dists) / opt_path # The MOR can be less than 1 because of the target's radius
        
        steps = [np.linalg.norm(dl) for dl in arm_traj[1:, :] - arm_traj[:-1, :] ]
        metricData['PO'][thisTrial] = sum(steps) / opt_path
        
        # Display a lot of curves to check the envelope of the raw data
#        goal_traj = np.array(arm_trajectory)
#        vals = np.array(vals)
#        forces = np.array(forces)
#        
#        fig = plt.figure()
#        cols = ['r', 'g', 'b']
#        lab = ['pX', 'pY', 'pZ', 'gX', 'gY', 'gZ', 'vX', 'vY', 'vZ', 'fX', 'fY', 'fZ']
#        
#        pax = fig.add_subplot(3, 1, 1)
#        for c in range(3):
#            pax.plot(times, goal_traj[:, c], cols[c] + '--', label = lab[c + 3])
#        plt.xlabel('Elapsed time (s)')
#        plt.ylabel('Distance (m)')
#        plt.legend(bbox_to_anchor=(1, 1), loc = 1, borderaxespad = 0.)
#        plt.title('Goal coordinates')
#        
#        vax = fig.add_subplot(3, 1, 2)
#        for c in range(3):
#            vax.plot(times, vals[:, c], cols[c], label = lab[c + 6])
#        plt.xlabel('Elapsed time (s)')
#        plt.ylabel('Speed (m/s)')
#        plt.legend(bbox_to_anchor=(1, 1), loc = 1, borderaxespad = 0.)
#        plt.title('Goal speed')
#        
#        fax = fig.add_subplot(3, 1, 3)
#        for c in range(3):
#            fax.plot(times, forces[:, c], cols[c], label = lab[c + 9])
#        plt.xlabel('Elapsed time (s)')
#        plt.ylabel('Voltage (V)')
#        plt.legend(bbox_to_anchor=(1, 1), loc = 1, borderaxespad = 0.)
#        plt.title('Force measurements')
        
        print('Reach routine ended.')
        
        ###################################################################################################################
        """
        Execution routine : <<< SAVE >>>
        """
        # Format force and trajectory data and save
        forceData = np.zeros(allAcqui.shape[1], dtype = forceDType)
        for ind, fname in enumerate(forceDNames):
            forceData[fname] = allAcqui[ind, :]
        np.savez(os.path.join(paramInit.dirSubject, 'Trial{}Force'.format(thisTrial + 1) ), forceData = forceData)
        
        trajData = np.zeros(arm_traj.shape[0], dtype = trajDType)
        for ind, tname in enumerate(trajDNames):
            trajData[tname] = arm_traj[:, ind]
        np.savez(os.path.join(paramInit.dirSubject, 'Trial{}Traj'.format(thisTrial + 1) ), trajData = trajData)
        
        print('Force and trajectory data saved for trial {}'.format(thisTrial + 1) )
    
    np.savez(os.path.join(paramInit.dirSubject, 'Metrics'), metricData = metricData)
    time.sleep(0.5)
    arm.shutdown()
#    plt.show()
    return paramInit.win#.close()
    #core.quit()
    
if __name__=='__main__':
    # Instanciate a Configuration object containing all the parameters values defining the experiment
    config = Configuration()
    
    # Subject and output folder info
    config.dirData = 'C:\Users\hybrid.NEUROCOG\Documents\coAdaptMyoelec\GripperUsability_exp1\data'
    config.subjectName = 'Toast'
    modality = 'C'
    
    # Data acquisition info
    config.nForcesRecorded = 3
    config.frequency = 1000
    config.nSamplesPerUpdate = 50 # The *approximate* update frequency of the control loop is frequency / nSamplesPerUpdate
    config.nSamplesPerBatch = 75
    
    # Data processing info
    config.instruction_duration = 4.0
    config.angular_offset = -2.35570894542 # [rad]    
    config.vel_threshold = 0.005 # [m/s]
    if modality == 'A':
        config.mode = 'pos'
        config.gain = 1.4 * np.array([-1, -1, 1])
    elif modality =='B':
        config.mode = 'vel'
        config.gain = 1.2 * np.array([-1, -1, 1])
    elif modality =='C':
        config.mode = 'vel2'
        config.gain = 35 * np.array([-1, -1, 1])
    else:
        raise ValueError('\'{}\' was not recognized as a modality: should be A, B or C'.format(modality) )
    
    # Task parameters
    config.init_pos = np.array([0.18, -0.14, 0.205]) # [m]
    config.init_pos_radius = 0.02 # [m]
    config.target_radius = 0.02 # [m]
    config.trialDelay = 25 # [s]
    config.dwellingTime = 0.6 # [s]
    
    root = Tk()
    config.screenWidth, config.screenHeight = root.winfo_screenwidth(), root.winfo_screenheight()
    root.destroy()
    
    funcGoalDriving(config)