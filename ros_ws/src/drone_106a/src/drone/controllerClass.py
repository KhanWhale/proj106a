#!/usr/bin/env python3

import rospy
import sys
import numpy as np
import time
import itertools
from collections import deque

SCALING_FACTOR = 20

inpMap = {}

inpMap['roll_max'] = 142.60744252895077
inpMap['roll_min'] = -974.0248326479165
inpMap['pitch_max'] = 48.54839867490042
inpMap['pitch_min'] = -7134.662853191837
inpMap['yaw_max'] = 31.20910636629924
inpMap['yaw_min'] = -501.24346088775826
inpMap['h_max'] = 205.17686522463777
inpMap['h_min'] = -119.56328012410148

# TODO: Consider using a rate if we're publishing to drone too quick - (don't expect this to be the case)
# will have to decouple state update from control loop then (right now state update and control loop are basically one)
class controllerClass(object):
    """
    A controller object

    # 4x' ndarray format: roll, pitch, yaw, height

    Fields:
    _Kp: 4x' ndarray of proportional constants
    _Ki: 4x' ndarray of integral constants
    _Kd: 4x' ndarray of derivative constants
    _Kth: 4x' ndarray of thresholds for integral term
    _heightScale: scaling between CV height and drone height
    _LastError: 4x' ndarray of previous position errors
    _LastTime: Time from start at which LastError was updated (in sec)
    _StartTime: Time when drone took off, set using obj._startTime = rospy.Time.now() (in sec) TODO
    _IntError: 4x' ndarray of integrated error values

    _times: For Plotting
    _actual_positions: For Plotting
    _actual_velocities: For Plotting
    _target_positions: For Plotting
    _target_velocities: For Plotting

    Methods:
    __init__(self, Kp, Ki, Kd, Kth): constructor

    """

    def __init__(self, Kp, Kd, Ki, Kth, heightScale):
        """
        Constructor:

        Inputs:
        Kp: 4x' ndarray of proportional constants
        Ki: 4x' ndarray of integral constants
        Kd: 4x' ndarray of derivative constants
        Kth: 4x' ndarray of thresholds for integral term
        heightScale: scaling between CV height and drone height
        """

        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd
        self._Kth= Kth
        self._heightScale = heightScale

        self._LastError = np.zeros(len(Kd))
        self._LastTime = 0
        self._IntError = np.zeros(len(Ki))
        
        # TODO: consider increasing ring buffer capacity
        self._ring_buff_capacity = 3
        self._ring_buff = deque([], self._ring_buff_capacity)

        # For Plotting:
        self._times = list()
        self._actual_positions = list()
        self._actual_velocities = list()
        self._target_positions = list()
        self._target_velocities = list()

    def set_start(self, time):
        self.startTime = float(time)
        return
    def control_wrapper(self, input, state, log=True):
        """
        Return command string for given input and state dicts

        Inputs:
        input: RAW dict of 'roll', 'pitch', 'yaw', 'height' (unscaled) TODO
        state: current state dict with lots of keys
        log: should the controller display a plot

        rc a b c d

        Set remote controller control via four channels.
        “a” = ROLL = left/right (-100-100)
        “b” = PITCH = forward/backward (-100-100) 
        “c” = HEIGHT = up/down (-100-100)
        “d” = YAW = yaw (-100-100)

        """
        t = time.time() - self.startTime
        # TODO: Once gestures have been passed, populate input dict correctly in controller.py and check here to decide command based on gesture 

        controlString = None
        
        scaledInput = input.copy()

        for key in input.keys():
            if key != 'gesture':
                scaledInput[key] = SCALING_FACTOR * (max(-input[key]/inpMap[key + '_min'], -1) if input[key] < 0 else min(input[key]/inpMap[key + '_max'], 1))


        scaledInput['h'] = scaledInput['h'] * self._heightScale / 10

        # Get the input for this time
        u = self.step_control(scaledInput, state, t)

        print(u)

        controlString = f'rc {int(u[0])} {int(u[1])} {0} {0}'

        # TODO: figure out how to reuse their plotting infra => we can just have 4 "limbs"

        if log:
            import matplotlib.pyplot as plt

            times = np.array(self._times)
            actual_positions = np.array(self._actual_positions)
            actual_velocities = np.array(self._actual_velocities)
            target_positions = np.array(self._target_positions)
            target_velocities = np.array(self._target_velocities)
            plt.figure()
            joint_num = len(self._path.joint_trajectory.joint_names)
            for joint in range(joint_num):
                plt.subplot(joint_num,2,2*joint+1)
                plt.plot(times, actual_positions[:,joint], label='Actual')
                plt.plot(times, target_positions[:,joint], label='Desired')
                plt.xlabel("Time (t)")
                if(joint == 0):
                    plt.ylabel(self._path.joint_trajectory.joint_names[joint] + " Position Error")
                else:
                    plt.ylabel(self._path.joint_trajectory.joint_names[joint])
                plt.legend()

                plt.subplot(joint_num,2,2*joint+2)
                plt.plot(times, actual_velocities[:,joint], label='Actual')
                plt.plot(times, target_velocities[:,joint], label='Desired')
                plt.xlabel("Time (t)")
                if(joint == 0):
                    plt.ylabel(self._path.joint_trajectory.joint_names[joint] + " Velocity Error")
                else:
                    plt.ylabel(self._path.joint_trajectory.joint_names[joint])
                plt.legend()

            print("Close the plot window to continue")
            plt.show()

        return controlString

    def step_control(self, input, state, t):
        """
        Return the control input given the current controller state at time t

        Inputs:
        input: RAW dict of 'roll', 'pitch', 'yaw', 'height' **SCALED** TODO
        state: current state dict with lots of keys
        t: time from start in seconds

        Output:
        u: 4x' ndarray of inputs a b c d in "rc a b c d" command
        
        """
        print(f'state = {state}')
        rospy.loginfo(f'state = {state}')
        print(f'input = {input}')
        rospy.loginfo(f'input = {input}')

        current_position = np.array([state[key] for key in ['roll', 'pitch', 'yaw', 'h']])

        target_position = np.array([input[key] for key in ['roll', 'pitch', 'yaw', 'h']])

        # For Plotting
        self._times.append(t)
        self._actual_positions.append(current_position)
        self._target_positions.append(target_position)

        # Error Term
        error = target_position - current_position

        print(f"error = {error}")

        # Integral Term
        # Value if below threshold
        potentialIntegralValue = self._IntError + error
        assignmentCondition = self._Kth > potentialIntegralValue 

        self._IntError[assignmentCondition] = potentialIntegralValue[assignmentCondition]
        self._IntError[~assignmentCondition] = self._Kth[~assignmentCondition]
        
        # Derivative Term
        rospy.loginfo(f"t : {t}")
        rospy.loginfo(f"t : {self._LastTime}")
        dt = t - self._LastTime
        
        print(f"LastError = {self._LastError}")
        print(f"dt = {dt}")
        
        # We implement a moving average filter to smooth the derivative
        curr_derivative = (error - self._LastError) / dt
        
        self._ring_buff.append(curr_derivative)
        ed = np.mean(self._ring_buff)

        # Save terms for the next run
        self._LastError = error
        self._LastTime = t

        ###################### YOUR CODE HERE #########################

        # Note, you should load the Kp, Ki, Kd, and Kw constants with
        # self._Kp
        # and so on. This is better practice than hard-coding

        u = self._Kp*error + self._Kd*ed  + self._Ki*self._IntError

        ###################### YOUR CODE END ##########################

        return u


if __name__ == "__main__":

    rospy.init_node('controllerClass', anonymous=True)

    print("controllerClass")

    try:
        rospy.spin()
    except (rospy.exceptions.ROSException, KeyboardInterrupt) as e:
        exit(0)