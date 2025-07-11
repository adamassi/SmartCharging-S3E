import unittest
from unittest.mock import MagicMock
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from semantic_questions import SmartChargingSemantic
from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor
import time
from sim_ur5.mujoco_env.common.ur5e_fk import forward
from battery_class import *


class MockBattery:
    def __init__(self, battery_type, charge=0, damaged=False):
        self.battery_type = battery_type
        self._charge = charge
        self._damaged = damaged

    def check_charge_progress(self):
        return self._charge

    def check_is_damaged(self):
        return self._damaged


class MockSimEnv:
    def get_valid_geometry_names(self):
        return ['AA_charger/bottom', 'AAA_charger/bottom', 'some_geom']

    def get_normal_force(self, charger_geom, other_geom):
        # Simulate force depending on test case
        if charger_geom == 'AA_charger/bottom' and other_geom == 'some_geom':
            return [0, 0, 1]  # Force detected ⇒ charger not free
        return [0, 0, 0]


class TestSmartChargingSemantic(unittest.TestCase):

    def setUp(self):
        self.sim_env = SimEnv()
        self.semantic = SmartChargingSemantic(self.sim_env, {})

    def test_is_charged_true(self):
        battery = MockBattery(battery_type='AA', charge=80)
        self.assertTrue(self.semantic.is_charged(battery))
        

    def test_is_charged_false(self):
        battery = MockBattery(battery_type='AA', charge=50)
        self.assertFalse(self.semantic.is_charged(battery))

    def test_is_compatible_with_true(self):
        battery = MockBattery(battery_type='AA')
        charger_name = 'battery_AA_charger'
        self.assertTrue(self.semantic.is_compatible_with(charger_name, battery))

    def test_is_compatible_with_false(self):
        battery = MockBattery(battery_type='AAA')
        charger_name = 'battery_AA_charger'
        self.assertFalse(self.semantic.is_compatible_with(charger_name, battery))

    def test_is_damaged_true(self):
        battery = MockBattery(battery_type='AA', damaged=True)
        self.assertTrue(self.semantic.is_damaged(battery))

    def test_is_damaged_false(self):
        battery = MockBattery(battery_type='AA', damaged=False)
        self.assertFalse(self.semantic.is_damaged(battery))

    def test_should_be_discarded_true(self):
        battery = MockBattery(battery_type='AA', damaged=True)
        self.assertTrue(self.semantic.should_be_discarded(battery))

    def test_should_be_discarded_false(self):
        battery = MockBattery(battery_type='AA', damaged=False)
        self.assertFalse(self.semantic.should_be_discarded(battery))

    def test_is_charger_free_for_true(self):
        battery = MockBattery(battery_type='AAA')
        self.assertTrue(self.semantic.is_charger_free_for(battery))

    def test_is_charger_free_for_false(self):
        battery = MockBattery(battery_type='AA')
        self.assertFalse(self.semantic.is_charger_free_for(battery))


if __name__ == '__main__':
    unittest.main()
