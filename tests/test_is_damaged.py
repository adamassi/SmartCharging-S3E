
"""
in this test we charge battery AA for 6 seconds and then check if it is damaged
start with 0% charge
whith
"""
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from semantic_questions import SmartChargingSemantic
from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor
from sim_ur5.mujoco_env.common.ur5e_fk import forward
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
executor.wait(10)  # Wait for the simulation to be ready



"""
    Test the is_damaged method when the battery is charged above 60%.
"""
# === Tests for is_damaged ===
print("================================")
print("Testing is_damaged functionality")
##is_damaged_false##
for battery in batteries.values():
    #TODO FIRAS FUNCTIONALITY
    print(f"{battery.name}:")
    print(f"is damaged: {semantic.is_damaged(battery)} (Charge: {battery.check_charge_progress()}%)")

executor.wait(30)
# charg battery AA
print("================================")
print("starting to charge battery AA")
sim_env.place_object_in_charger(batteries['battery_AA/1'],[-0.7, -0.75, 0.09])  # Place the AA battery in its charger
# batteries['battery_AA/1'].start_charging()  # Start charging the AA battery

sim_env.wait(1)  # Wait for the battery to charge
print("after 1 second of charging")
print("================================")
### tests_is_damaged_false##
for battery in batteries.values():
    #TODO FIRAS FUNCTIONALITY
    print(f"{battery.name}:")
    print(f"is damaged: {semantic.is_damaged(battery)} (Charge: {battery.check_charge_progress()}%)")
print("================================")
print("starting to charge battery AAA AND C AND D")
sim_env.place_object_in_charger(batteries['battery_AAA/1'],[-0.7, -0.9, 0.09])  # Place the AA battery in its charger
sim_env.place_object_in_charger(batteries['battery_D/1'],[-0.8, -0.9, 0.09])  # Place the AA battery in its charger
sim_env.place_object_in_charger(batteries['battery_C/1'],[-0.7, -0.65, 0.09])  # Place the AA battery in its charger

executor.wait(1000)  # Wait for the battery to charge

sim_env.wait(6)  # Wait for the battery to charge
print("after 6 seconds of charging")
print("================================")
### tests_is_damaged_true##
for battery in batteries.values():
    print(f"{battery.name}:")
    print(f"is damaged: {semantic.is_damaged(battery)} (Charge: {battery.check_charge_progress()}%)")




sim_env.wait(80)  # Wait for the battery to charge

print("after 16 seconds of charging")