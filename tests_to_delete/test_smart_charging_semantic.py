import unittest
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from semantic_questions import SmartChargingSemantic
from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor
from sim_ur5.mujoco_env.common.ur5e_fk import forward
from battery_class import *

class TestSmartChargingSemantic(unittest.TestCase):
    """
    Unit tests for the SmartChargingSemantic class.
    """

    def setUp(self):
        """
        Set up the test environment.
        Creates a simulation environment and an instance of SmartChargingSemantic.
        """
        self.sim_env = SimEnv()  # Initialize the simulation environment
        joint_names = self.sim_env.get_all_joint_names()  # Get all joint names from the simulation environment
        # Create Battery instances for joints containing "battery" in their name
        self.batteries = {}
        for joint_name in joint_names:
            if "battery" in joint_name.lower():
                key = joint_name.split('-')[0]
                battery = Battery(name=joint_name, env=self.sim_env)
                self.batteries[key] = battery  # Store the battery instance in the dictionary
        

        self.semantic = SmartChargingSemantic(self.sim_env, {})  # Initialize the semantic class with an empty battery dictionary
######################
##TESTS FOR IF IS CHARGED
######################
    def test_is_charged_true(self):
        """
        Test the is_charged method when the battery is charged above 60%.
        """
        battery = Battery(battery_type='AA', charge=80)  # Create a battery with 80% charge
        self.assertTrue(self.semantic.is_charged(battery))  # Assert that the battery is considered charged

    def test_is_charged_false(self):
        """
        Test the is_charged method when the battery is charged below 60%.
        """
        battery = Battery(battery_type='AA', charge=50)  # Create a battery with 50% charge
        self.assertFalse(self.semantic.is_charged(battery))  # Assert that the battery is not considered charged

    def test_is_compatible_with_true(self):
        """
        Test the is_compatible_with method when the charger is compatible with the battery.
        """
        battery = Battery(battery_type='AA')  # Create a battery of type 'AA'
        charger_name = 'battery_AA_charger'  # Define a compatible charger name
        self.assertTrue(self.semantic.is_compatible_with(charger_name, battery))  # Assert compatibility

    def test_is_compatible_with_false(self):
        """
        Test the is_compatible_with method when the charger is not compatible with the battery.
        """
        battery = Battery(battery_type='AAA')  # Create a battery of type 'AAA'
        charger_name = 'battery_AA_charger'  # Define an incompatible charger name
        self.assertFalse(self.semantic.is_compatible_with(charger_name, battery))  # Assert incompatibility



if __name__ == '__main__':
    unittest.main()
