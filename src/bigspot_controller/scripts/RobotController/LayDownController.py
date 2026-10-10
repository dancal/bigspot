#!/usr/bin/env python3
#Author: lnotspotl
#Modified: Add LayDownController for smooth animation from lying position

import rospy
import numpy as np
import time
from RobotHardwares.calibration import CROUCH_HEIGHT_M, STANDING_HEIGHT_M

class LayDownController(object):
    """
    Controller for transitioning from lying down position to standing position
    This controller provides smooth animation with easing function
    """
    
    def __init__(self, default_stance):
        self.def_stance = default_stance
        
        # Animation parameters
        self.STEP = 0.05  # Smaller step for smoother animation
        self.STEP_MAX = 30  # More steps for smooth transition (10 -> 30)
        self.current_step = 0
        
    def updateStateCommand(self, msg, state, command):
        """
        LayDown state ignores joystick input
        Robot must complete standing up animation before accepting commands
        """
        pass
    
    @property
    def default_stance(self):
        a = np.copy(self.def_stance)
        return a
    
    def smoothstep_easing(self, t):
        """
        Smoothstep easing function for natural acceleration/deceleration
        Creates smooth animation curve instead of linear interpolation
        
        Args:
            t: normalized progress (0.0 to 1.0)
        
        Returns:
            eased progress value with smooth curve
        """
        # Smoothstep: 3t^2 - 2t^3
        return t * t * (3.0 - 2.0 * t)
    
    def run(self, state, command):
        """
        Main animation loop - gradually lifts robot from ground to standing position
        
        Args:
            state: Robot state object
            command: Robot command object
        
        Returns:
            foot_locations: Target foot positions for IK solver
        """
        temp = self.default_stance
        
        # Hold a reachable crouch until an explicit start command. This height
        # requires physical calibration; zero height is not an initial foot pose.
        temp[2] = [command.robot_height * (CROUCH_HEIGHT_M/STANDING_HEIGHT_M)] * 4
        
        state.foot_locations = temp
        return state.foot_locations
