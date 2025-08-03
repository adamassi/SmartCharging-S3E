import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
from semantic_questions import SmartChargingSemantic
from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor
from battery_class import Battery
from datetime import date

today = date.today()
# === Setup ===
# Initialize the simulation environment and motion executor
sim_env = SimEnv()
executor = MotionExecutor(sim_env)

# Create a dictionary to hold battery objects
batteries = {}

# Get all joint names from the simulation environment
joint_names = sim_env.get_all_joint_names()
for joint_name in joint_names:
    if "battery" in joint_name.lower():  # Filter for battery-related joints
        key = joint_name.split('-')[0]  # Extract the key (e.g., 'battery_AA')
        battery = Battery(name=joint_name, env=sim_env)  # Create a Battery instance
        batteries[key] = battery  # Add the battery to the dictionary

# Initialize the semantic reasoning class
semantic = SmartChargingSemantic(sim_env, batteries)

# Wait for the simulation to stabilize
executor.wait(100)

# Change 1: Charge battery_AA enough to make it "charged"
batt_AA = batteries['battery_AA/1']
print("== Starting to charge battery_AA ===")
sim_env.place_object_in_charger(batt_AA, [-0.7, -0.75, 0.09])  # Place battery_AA in its charger
executor.wait(100)

sim_env.wait(2)  # Wait long enough for the charge to exceed 60%
batt_AA.stop_charging()  # Stop charging battery_AA

# assert batt_AA.check_is_damaged() == "True"  # Ensure the battery is charged
print("== Finished charging battery_AA ===")
#TO DEBUG print the progress of the battery
print(f"Battery AA charge progress: {batt_AA.check_charge_progress()}%")

print("============================")
#intalize goal state is Battery AA is charged 
goal= {"IsCharged(battery_AA/1-"+str(today)+"/)","IsCharged(battery_D/1-"+str(today)+"/)"}

# Get the current state from the semantic reasoning class
state_filled = semantic.get_state()


# Calculate the success score
scor = semantic.success_score(state_filled, goal)
print(f"Success Score: {scor}")
# print(state_filled)
#test if battery damaged the score is 0 for it 

batt_AAA = batteries['battery_AAA/1']

sim_env.place_object_in_charger(batt_AAA, [-0.7, -0.9, 0.09])  # Place battery_AAA in its charger
print("== Starting to charge battery_AAA ===")
executor.wait(100)

sim_env.wait(11)  # Wait long enough for the battery to become damaged

batt_AAA.stop_charging()  # Stop charging battery_AAA

print("== Finished charging battery_AAA ===")

print("==Remove battery_AAA from charger===")
sim_env.remove_object_from_charger(batt_AAA.name, [-0.55, 0.0, 0.03]) 

goal= {"IsCharged(battery_AA/1-"+str(today)+"/)","IsCharged(battery_D/1-"+str(today)+"/)"}

# Get the current state from the semantic reasoning class
state_filled = semantic.get_state()


# Calculate the success score
goal= {"IsCharged(battery_AAA/1-"+str(today)+"/)"}


state_filled = semantic.get_state()


scor = semantic.success_score(state_filled, goal)
print(f"Success Score: {scor}")

sim_env.wait(3)