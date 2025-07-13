"""
Test for the is_charger_free_for method in the SmartChargingSemantic class.
This test checks if the charger is free for a given battery type.
"""
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from semantic_questions import SmartChargingSemantic
from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor
from battery_class import *

# === Setup ===
sim_env = SimEnv()
executor = MotionExecutor(sim_env)
batteries = {}
joint_names = sim_env.get_all_joint_names()  # Get all joint names from the simulation environment
for joint_name in joint_names:
    if "battery" in joint_name.lower():
        key = joint_name.split('-')[0]  # Get the key without the trailing slash
        print(key)
        battery = Battery(name=joint_name, env=sim_env)
        batteries[key] = battery
        print(f"Created Battery instance for joint: {joint_name}")

semantic = SmartChargingSemantic(sim_env, batteries)
executor.wait(100)  # Wait for the simulation to be ready

# === Tests for is_charger_free_for ===
print("================================")
print("Testing is_charger_free_for functionality")

# Initial state: All chargers should be free
for battery in batteries.values():
    print(f"{battery.name}:")
    print(f"Is charger free for battery type {battery.battery_type}: {semantic.is_charger_free_for(battery)}")

# Place a battery on its charger
print("================================")
print("Placing battery AA on its charger")
sim_env.place_object_in_charger(batteries['battery_AA/1'], [-0.7, -0.75, 0.02])  # Place the AA battery in its charger

executor.wait(10)  # Wait for the simulation to update
print("After placing battery AA on its charger")
for battery in batteries.values():
    print(f"{battery.name}:")
    print(f"Is charger free for battery type {battery.battery_type}: {semantic.is_charger_free_for(battery)}")


print("================================")
print("Placing battery AAA on its charger")
sim_env.place_object_in_charger(batteries['battery_AAA/1'], [-0.7, -0.9, 0.09])

executor.wait(10)
print("After placing battery AAA on its charger")
for battery in batteries.values():
    print(f"{battery.name}:")
    print(f"Is charger free for battery type {battery.battery_type}: {semantic.is_charger_free_for(battery)}")


print("================================")
print("Removing battery AA from its charger")
sim_env.remove_object_from_charger(batteries['battery_AA/1'].name,[-0.55, 0.0, 0.04])

executor.wait(1)
print("After removing battery AA from its charger")
for battery in batteries.values():
    print(f"{battery.name}:")
    print(f"Is charger free for battery type {battery.battery_type}: {semantic.is_charger_free_for(battery)}")

# Place another battery on a different charger
print("================================")
print("Placing battery AAA on its charger")
sim_env.place_object_in_charger(batteries['battery_AAA/1'], [-0.7, -0.9, 0.09])  # Place the AAA battery in its charger

executor.wait(10)  # Wait for the simulation to update
print("After placing battery AAA on its charger")
for battery in batteries.values():
    print(f"{battery.name}:")
    print(f"Is charger free for  {battery.battery_type}: {semantic.is_charger_free_for(battery)}")

# Remove a battery from its charger
print("================================")
print("Removing battery AA from its charger")

executor.wait(1)  # Wait for the simulation to update
sim_env.wait(5)