# -*- coding: utf-8 -*-
"""
.. module:: gripperarm
This module implements the GripperArm class, designed as a way to use the IKPy module with a robot based on Poppy Right Arm
"""


# Connection and mechanical model of the robot
from poppy.creatures import PoppyRightGripper
from poppy_right_gripper.config import URDFpath_full, URDFpath_noclamp, JSONpath_full, JSONpath_noclamp
from ikpy.chain import Chain

# Plotting utilities
from ikpy import plot_utils
import matplotlib.pyplot as plt

# Miscellaneous
from time import sleep
from numpy.linalg import norm
import numpy as np


class GripperArm(object):
    """High-level class to operate simultaneously ikpy and pypot features in the case of the Poppy Gripper Arm robot.
    The two main differences in the ways IKPy and PyPot manage data are
    This class can operates two different versions of the robotic arm: with or without the two-motor wrist-and-clamp assembly
    """
    def __init__(self, version = 'full'):
        """Instanciates both a pypot.Robot and a ikpy.Chain corresponding to the version of the Poppy Gripper Arm.
        :param string version: Optional: Version of the robot to instanciate.
            'noclamp' corresponds to a version whithout the gripper motors and clamps, and the endpoint should be placed at the end of a stick fixed to the forearm
            Any other parameter corresponds to a full version where every motor is enabled and the endpoint is placed along the wrist's rotation axis, at the level of the grasping area
        :returns: The list of angular positions
        """
        self.version = 'full' if version != 'noclamp' else 'noclamp'
        sleep(0.5) # Short pause to allow the robot connections to fully load

        # Import URDF to build the kinematic chain
        if version == 'noclamp':
            URDFpath = URDFpath_noclamp
            JSONpath = JSONpath_noclamp
        else:
            URDFpath = URDFpath_full
            JSONpath = JSONpath_full
        
        self.robot = PoppyRightGripper(config = JSONpath)
        self.chain = Chain.from_urdf_file(URDFpath, active_links_mask = [False, True, True, True, True, False, False])
        
        # Change PID of arm motors MX-28
        for m in filter(lambda m: hasattr(m, 'pid'), self.robot.r_arm):
            m.pid = (1.9, 5, 0)
        if self.version != 'noclamp':
            # Change PID of wrist motor XL-320
            self.robot.r_wrist_z.pid = (4, 2, 0)
            # Change PID of gripper motor XL-320
            self.robot.r_gripper.pid = (8, 0, 0)
        # Reduce max torque to keep motor temperature low
        for m in self.robot.motors:
            m.torque_limit = 70
        
        # Calculate the shoulder position and the total arm length to define the sphere including all the reachable space
        self.shoulder_origin = np.ravel( np.dot(self.chain.links[1].get_transformation_matrix(0), self.chain.links[2].get_transformation_matrix(0) )[:3, 3])
        self.length = sum([l._length for l in self.chain.links[3:] ]) * 0.99 # Rough approximation of the arm's length
        
        # Define the regularization parameters corresponding to each link: factor and ideal angle
        reg_factor = 0.01 * np.array([0, 5, 4.5, 1.8, 1, 0, 0])
        reg_angles = np.radians([0., 0., 0., 0., -80., 0., 0.])
        self.regularization_parameters = (reg_factor, reg_angles)

        # Create a new list of motors in order to sync ikpy Links with pypot Motors along the kinematic chain.
        self.motors = []
        motors_names = [m.name for m in self.robot.motors]
        nMotors = len(motors_names) if self.version == 'noclamp' else len(motors_names) - 1 # The clamp motor is ignored in full version
        for l in self.chain.links[1:(nMotors + 1)]: # The base link isn't taken into account
            try:
                self.motors.append(self.robot.motors[motors_names.index(l.name)])
            except (ValueError):
                raise ValueError('Failed to find a motor named "{}". Check for discrepancies between\n{} and\n{}'.format(l.name, JSONpath, URDFpath) )

        # Import the angular limits of the motors
        self.lower_limits = [min(m.lower_limit, m.upper_limit) for m in self.motors]
        self.upper_limits = [max(m.lower_limit, m.upper_limit) for m in self.motors]
        
        # Set at standard starting position
        self.robot.compliant = False
        self.robot.power_up()
        self.init_angles()
        
        print('Gripper arm initialized and running...')
        
    def get_positions(self, unit = 'deg', space = 'link'):
        """Gets and returns the current angular positions of the robot.
        :param string unit: Optional: Unit of the output angles. 'deg' for degrees, anything else for radians
        :param string space: Optional: Formal space of the output angles. 'link' for a full length list, anything else for a list restricted to the kinematic chain's motors
        :returns: The list of angular positions of all links
        """
        pos = [] if space != 'link' else [0] # Base link angular position is always zero
        for m in self.motors:
            pos.append(m.present_position) # 4 angles for noclamp version, 5 angles for full version
        
        if self.version != 'noclamp':
            pos.append(self.robot.r_wrist_z.present_position)
        
        if space == 'link':
            pos.extend([0] * (len(self.chain.links) - len(pos) ) )
        
        # pypot works with degrees whereas ikpy works with radians
        if unit != 'deg':
            return np.radians(pos)
        else:
            return np.array(pos)

    def get_endpoint_position(self):
        """Gets and returns the current position of the robot's endpoint.
        :returns: The 3-D coordinates of the endpoint
        """
        return self.chain.forward_kinematics(self.get_positions(unit = 'rad', space = 'link') )[:3, 3]
        
    def goto_positions(self, target_angles, duration, wait = False):
        """Moves the robot's joints to the given target angles. The clamp motor is ignored in the case of a full version.
        If the given angles exceed the motor bounds, moves to the closest permitted angles and warns the user.
        :param list target_angles: List of angular positions to reach, in degrees
        :param float duration: Duration of the movement in seconds
        :param bool wait: Optional: Whether or not the method waits for the movement to finish before exiting
        """

        # Check if the number of angles is correct
        nMotors = len(self.motors)
        if len(target_angles) != nMotors:
            raise ValueError('Invalid target angles size: {} values expected, {} values given.'.format(nMotors, len(target_angles) ) )
        
        warning = False
        wnames = []
        for m, a, ll, ul in zip(self.motors, target_angles, self.lower_limits, self.upper_limits):
            m.goto_position(a, duration)
            # Check if target angular position is within motor limits
            if (a < ll) or (a > ul):
                warning = True
                wnames.append(m.name)
        
        if warning:
            print('WARNING: target angles exceeded angular limits for motors:\n{}'.format(wnames))
            
        if wait:
            sleep(duration)
    
    def set_motor_goals(self, target_angles = []):
        """Sets the motor goals to the given target angles. If no target angles are given, the motor goals are set to their current positions.
        :param list target_angles: Optional: list of angular positions to reach by the motors, in degrees
        """
        if target_angles == []: # If no target given, get current motor positions
            target_angles = self.get_positions(unit = 'deg', space = 'motor')
        
        for m, a in zip(self.motors, target_angles):
            m.goal_position = a
    
    def set_clamp_position(self, target_angle, duration, wait = False):
        """Moves the gripper joint to the given target angle.
        If the given angle exceeds the motor bounds, moves to the closest permitted angle and warns the user.
        :param float target_angle: Angular position to reach, in degrees
        :param float duration: Duration of the movement, in seconds
        :param bool wait: Optional: Whether or not the method waits for the movement to finish before exiting
        """
        if self.version == 'noclamp':
            print('The current robot was created as \'noclamp\' version!')
        else:
            self.robot.r_gripper.goto_position(target_angle, duration, wait = wait)

    def init_angles(self):
        """Sets the motor positions at standard angles in order to prevent collision when the robot is turned on, and unsafe falling when turned off.
        """
        angles = [0, 0, 0, -7] if self.version == 'noclamp' else [0, 0, 0, -7, 0]
        self.goto_positions(angles, 1.5, wait = True)

    def solve_ik(self, target_pos, initial_pos = None):
        """Computes the angular positions corresponding to the position of the robot's endpoint matching best with the target position
        :param list target_pos: 3-D target position to reach, in metre
        :param list initial_pos: Optional: The initial angular positions of each joint of the chain. Defaults to 0 for each joint
        :returns: A two-element tuple consisting of the full chain angles (7 values in rad) and the motor angles (4 or 5 values in deg)
        """
        target_frame = np.eye(4) # ikpy solver requires a full 4x4 transformation matrix
        target_frame[:3, 3] = target_pos
        
        # Manage the initial position from which to compute the IK, and convert from pypot degrees to ikpy radians
        if initial_pos == 'current':
            initial_pos = self.get_positions(unit = 'rad', space = 'link')
        elif initial_pos != None:
            initial_pos = np.radians(initial_pos)
        
        # Compute the inverse kinematics from the given initial position with:
        # * a maximum of 14 iterations
        # * an error tolerance of 0.25 mm
        #   this accuracy being purely theoretical: the physical robot would never reach such an excellent accuracy
        # These parameters allow for a correct accuracy (~ 2-10 mm on a physical robot) with a reasonable processing time.
        ref_angles = self.chain.inverse_kinematics(target_frame, initial_position = initial_pos, max_iter = 14, tol = 2.5*1e-4, reg = self.regularization_parameters )
        # Replace non relevant angles with zeros on the full length list
        res_angles = map(lambda ang, mask: ang if mask else 0. , ref_angles, self.chain.active_links_mask)
        
        # Get only the angles corresponding to the motors of the kinematic chain
        # N.B. : the virtual "Base link" is never included
        if self.version != 'noclamp':
            motor_angles = res_angles[1:-1] # Don't include the clamp motor
        else:
            motor_angles = res_angles[1:-2]
            
        # Return
        # * full list of angles in rad, used for plotting the result
        # * list of motor angles in deg, used to control the robot       
        return (res_angles, np.degrees(motor_angles) )

    def goto_target(self, target_pos, duration = None, initial_pos = 'current', wait = False):
        """Moves the robot to the target position only if it's reachable. If not, warns the user.
        :param list target_pos: 3-D target position to reach, in metre
        :param float duration: Optional: Duration of the movement, in seconds. If none is given, uses motor.goal_position attribute rather than motor.goto_position method
        :param bool wait: Optional: Whether or not the method waits for the movement to finish before exiting
        :returns: The same tuple as solve_ik, corresponding to the reached position if target is reachable or the current joint positions if it isn't
        """
        target_ok = True
        if (norm(target_pos - self.shoulder_origin) > self.length): # Restrict to a sphere centered on the shoulder
            target_ok = False
            err = 'too far from the shoulder.'
        elif (norm(target_pos - self.shoulder_origin) < 0.16): # Exclude a sphere centered on the shoulder
            target_ok = False
            err = 'too close to the shoulder.'
        elif (target_pos[1] > 0.135): # Exclude far left area
            target_ok = False
            err = 'too far left.'
        elif (target_pos[1] > 2. * target_pos[0] - 0.065): # Exclude the left area that could make the robot hit the vertical pole
            target_ok = False
            err = 'risks of hitting the vertical pole with the arm.'
        elif (norm(target_pos[:2]) < 0.0015): # Exclude cylinder along the vertical pole
            target_ok = False
            err = 'risks of hitting the vertical pole with the effector.'
        
        if target_ok:
            res = self.solve_ik(target_pos, initial_pos = initial_pos)
            if duration == None:
                self.set_motor_goals(res[1])
            else:
                self.goto_positions(res[1], duration, wait = wait)
        else:
            print('Target out of reachable space: ' + err)
            res = (-1, -1)
        
        return res
    
    def shutdown(self):
        """Standard way to shut down the robot before unplugging the power source or exiting a script.
        Moves the robot to standard idle position, make it compliant, then close the connections.
        """
        print('Shutting down the robot...')
        
        self.init_angles()
        self.robot.compliant = True
        self.robot.close()
        
        print('Robotic arm shut down. Power source can now be safely unplugged.')


if __name__ == '__main__': # A basic demonstration, presenting various features available with this class
    # Full robot
    arm = GripperArm('full')
    
    ax = plot_utils.init_3d_figure()
    
    # Reach first target, then close gripper
    target = [0.215, -0.13, 0.3]
    (fa, ma) = arm.goto_target(target, 1.5, wait = True)
    th_epos = arm.chain.forward_kinematics(fa)[:3, 3]
    current_epos = arm.get_endpoint_position()
    print('\nTheoretical error to target [m]: {}\nActual error to target [m]: {}'.format(norm(target - th_epos), norm(target - current_epos) ) )
    arm.chain.plot(fa, ax, target = target)
    arm.set_clamp_position(25, 1, wait = True)
    
    # Reach second target, then open gripper
    target = [0.22, -0.09, 0.24]
    (fa, ma) = arm.goto_target(target, 1.5, wait = True)
    th_epos = arm.chain.forward_kinematics(fa)[:3, 3]
    current_epos = arm.get_endpoint_position()
    print('\nTheoretical error to target [m]: {}\nActual error to target [m]: {}'.format(norm(target - th_epos), norm(target - current_epos) ) )
    arm.chain.plot(fa, ax, target = target)
    arm.set_clamp_position(-15, 1, wait = True)
    
    print('\nClose the figure window to continue.')    
    plt.show()
    arm.shutdown()
    
    # Robot without clamp
    arm = GripperArm(version = 'noclamp')
    ax = plot_utils.init_3d_figure()
    
    # Reach fourth target
    target = [0.19, -0.12, 0.19]
    (fa, ma) = arm.goto_target(target, 1.5, wait = True)
    arm.chain.plot(fa, ax, target = target)
    ax.set_xlim(-0.1, 0.4)
    ax.set_ylim(-0.35, 0.15)
    ax.set_zlim(0, 0.5)
    ax.plot([target[0], target[0], target[0]], [target[1], target[1], 0], [target[2], 0, 0], 'r--')
    
    print('\nClose the figure window to exit.')    
    plt.show()
    arm.shutdown()