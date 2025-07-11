"""
In this test, we charge battery AA for 6 seconds and then check if it should be discarded.
Start with 0% charge.
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

# === Tests for should_be_discarded ===
print("================================")
print("Testing should_be_discarded functionality")
# Initial state: No battery should be discarded
for battery in batteries.values():
    print(f"{battery.name}:")
    print(f"Should be discarded: {semantic.should_be_discarded(battery)} (Charge: {battery.check_charge_progress()}%)")

# Start charging battery AA
print("================================")
print("Starting to charge battery AA")
batteries['battery_AA/1'].start_charging()  # Start charging the AA battery

sim_env.wait(1)  # Wait for 1 second
print("After 1 second of charging")
for battery in batteries.values():
    print(f"{battery.name}:")
    print(f"Should be discarded: {semantic.should_be_discarded(battery)} (Charge: {battery.check_charge_progress()}%)")

# Start charging other batteries
print("================================")
print("Starting to charge batteries AAA, C, and D")
batteries['battery_AAA/1'].start_charging()  # Start charging the AAA battery
batteries['battery_C/1'].start_charging()  # Start charging the C battery
batteries['battery_D/1'].start_charging()  # Start charging the D battery

sim_env.wait(6)  # Wait for 6 seconds
print("After 6 seconds of charging")
for battery in batteries.values():
    print(f"{battery.name}:")
    print(f"Should be discarded: {semantic.should_be_discarded(battery)} (Charge: {battery.check_charge_progress()}%)")

sim_env.wait(10)  # Wait for additional time
print("After 16 seconds of charging")
for battery in batteries.values():
    print(f"{battery.name}:")
    print(f"Should be discarded: {semantic.should_be_discarded(battery)} (Charge: {battery.check_charge_progress()}%)")