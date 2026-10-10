"""Checks command ordering and rejection using driver substitutes, never real motors."""
import importlib
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/bigspot_controller/scripts'))


class HardwareBoundaryTests(unittest.TestCase):
    def setUp(self):
        drivers={'board':types.SimpleNamespace(SCL=None,SDA=None),
                 'adafruit_pca9685':types.SimpleNamespace(PCA9685=None),
                 'adafruit_servokit':types.SimpleNamespace(ServoKit=None)}
        with patch.dict(sys.modules,drivers):
            self.hardware=importlib.import_module('RobotHardwares.RobotHardwares')
        self.controller=self.hardware.ServoController.__new__(self.hardware.ServoController)
        self.writes=[]
        def item(name):
            return types.SimpleNamespace(posName=name,defAngle=90,direction=1,restPos=0,
                rad2deg=lambda v:v*180/np.pi,
                moveAngle=lambda v:self.writes.append((name,v)))
        self.controller.servoMoters=[item(n) for n in ['FLS','FLL','FLF','FRS','FRL','FRF','RLS','RLL','RLF','RRS','RRL','RRF']]

    def test_original_ik_slot_correspondence_is_preserved(self):
        self.controller.move(np.arange(12)*.01,None)
        self.assertEqual([n for n,v in self.writes],['FLS','FLL','FLF','FRS','FRL','FRF','RLS','RLL','RLF','RRS','RRL','RRF'])
        np.testing.assert_allclose([v for n,v in self.writes],np.arange(12)*.01)

    def test_invalid_batch_writes_no_partial_motor_commands(self):
        for invalid in [np.full(12,np.nan),np.full(12,4.),np.zeros(11)]:
            with self.assertRaises(ValueError):
                self.controller.move(invalid,None)
            self.assertEqual(self.writes,[])
