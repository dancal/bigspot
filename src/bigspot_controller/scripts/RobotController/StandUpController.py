#!/usr/bin/env python3
#Author: lnotspotl
#Modified: Improved StandUpController with smoother animation

import rospy
import numpy as np
import time
import threading

from RoboticsUtilities.Transformations import rotxyz

class StandUpController(object):
    """
    Controller for standing up animation with smooth easing
    Transitions from partially lying to fully standing position
    """

    def __init__(self, default_stance):
        self.def_stance = default_stance
        self.max_reach  = 0.065

        self.FR_X       = 0.
        self.FR_Y       = 0.
        self.FL_X       = 0.
        self.FL_Y       = 0.

        # Improved animation parameters for smoother motion
        self.STEP       = 0.05  # Reduced from 0.15 for finer control
        self.STEP_MAX   = 30    # Increased from 10 for smoother animation (3x more frames)
        self.ISDOWN     = False

    def updateStateCommand(self, msg, state, command):
        """Handle joystick input during stand-up animation"""
        self.FR_X       = msg.axes[1]
        self.FR_Y       = msg.axes[0]

        self.FL_X       = msg.axes[4]
        self.FL_Y       = msg.axes[3]
    
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
        
    @property
    def default_stance(self):
        a = np.copy(self.def_stance)
        return a

    def run(self, state, command):
        """
        Main animation loop - smoothly transition to standing position
        
        Args:
            state: Robot state object
            command: Robot command object
        
        Returns:
            foot_locations: Target foot positions for IK solver
        """

        temp        = self.default_stance
        if state.ticks == 0:
            state.ticks = 1

        # Calculate animation progress (0.0 to 1.0)
        progress = min(state.ticks / self.STEP_MAX, 1.0)
        
        # Apply easing function for smooth animation
        eased_progress = self.smoothstep_easing(progress)
        
        # Interpolate height from 0 to command.robot_height
        temp[2] = [(command.robot_height * eased_progress)] * 4
        
        # Increment animation counter
        state.ticks  += self.STEP
        if state.ticks > self.STEP_MAX:
            state.ticks  = self.STEP_MAX
        else:
            # Small sleep for visual smoothness (non-blocking alternative to time.sleep recommended)
            time.sleep(0.02)

        state.foot_locations = temp
        return state.foot_locations
