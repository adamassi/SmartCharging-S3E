
"""
in this test we charge battery AA for 2 seconds and then check if it is charged
and chck the charge progress
start with 0% charge
whith
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
        key= joint_name.split('-')[0]  # Get the key without the trailing slash
        print(key)
        battery = Battery(name=joint_name,env=sim_env)
        batteries[key] = battery
        print(f"Created Battery instance for joint: {joint_name}")

semantic= SmartChargingSemantic(sim_env, batteries)
executor.wait(100)  # Wait for the simulation to be ready

sim_env.place_object_in_charger(batteries['battery_AA/1'],[-0.7, -0.75, 0.09])  # Place the AA battery in its charger
print("will select battery_AA/1")
# sim_env.select_body("battery_AA/")
sim_env.select_free_joint("battery_AA/1-2025-07-13/")
executor.wait(5000)
"""
    Test the is_charged method when the battery is charged above 60%.
"""
# === Tests for is_charged ===
print("================================")
print("Testing is_charged functionality")
##is_charged_false##
for battery in batteries.values():
    #TODO FIRAS FUNCTIONALITY
    print(f"{battery.name}:")
    print(f"is charged: {semantic.is_charged(battery)} (Charge: {battery.check_charge_progress()}%)")

# sim_env.place_object_in_charger(batteries['battery_AA/1'].name,[-0.7, -0.75, 0.09])  # Place the AA battery in its charger
executor.wait(30)
# charg battery AA
print("================================")
print("starting to charge battery AA")
# batteries['battery_AA/1'].start_charging()  # Start charging the AA battery

sim_env.wait(1)  # Wait for the battery to charge
print("after 1 second of charging")
print("================================")
### tests_is_charged_true##
for battery in batteries.values():
    #TODO FIRAS FUNCTIONALITY
    print(f"{battery.name}:")
    print(f"is charged: {semantic.is_charged(battery)} (Charge: {battery.check_charge_progress()}%)")

sim_env.wait(1)  # Wait for the battery to charge
print("after 2 seconds of charging")
print("================================")
### tests_is_charged_true##
for battery in batteries.values():
    #TODO FIRAS FUNCTIONALITY
    print(f"{battery.name}:")
    print(f"is charged: {semantic.is_charged(battery)} (Charge: {battery.check_charge_progress()}%)")

sim_env.wait(1)  # Wait for the battery to charge
print("================================")
print("starting to charge battery AAA")
sim_env.place_object_in_charger(batteries['battery_AAA/1'],[-0.7, -0.9, 0.05])  # Place the AAA battery in its charger



for battery in batteries.values():
    print(f"{battery.name}:")
    print(f"is charged: {semantic.is_charged(battery)} (Charge: {battery.check_charge_progress()}%)")

print(f"Charger free status: {semantic.is_charger_free_for(batteries['battery_AAA/1'])}")  # Check if the charger for AAA battery is free

print(f"Charger free status: {semantic.is_charger_free_for(batteries['battery_AA/1'])}")  # Check if the charger for AA battery is free

sim_env.wait(1000)  # Wait for the battery to charge

