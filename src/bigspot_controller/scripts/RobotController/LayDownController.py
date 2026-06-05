#!/usr/bin/env python3
#Author: lnotspotl
#Modified: Add LayDownController for smooth animation from lying position

import rospy
import numpy as np
import time

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
        
        # Initialize on first call
        if state.ticks == 0:
            state.ticks = 1
        
        # Calculate animation progress (0.0 to 1.0)
        progress = min(state.ticks / self.STEP_MAX, 1.0)
        
        # Apply easing function for smooth animation
        eased_progress = self.smoothstep_easing(progress)
        
        # Interpolate from lying position (0) to standing position (command.robot_height)
        target_height = command.robot_height * eased_progress
        temp[2] = [target_height] * 4
        
        # Increment animation progress
        state.ticks += self.STEP
        
        # Clamp to maximum
        if state.ticks > self.STEP_MAX:
            state.ticks = self.STEP_MAX
        else:
            # Optional: add small sleep for smooth visual animation
            time.sleep(0.02)
        
        state.foot_locations = temp
        return state.foot_locations
