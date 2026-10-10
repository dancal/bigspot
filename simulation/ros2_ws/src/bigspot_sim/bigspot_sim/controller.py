"""Compatibility entry point for the startup/IMU/watchdog controller."""
from .posture_node import Controller, main

__all__ = ['Controller', 'main']
