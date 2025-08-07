import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from semantic_questions import SmartChargingSemantic
from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor
from battery_class import Battery
"""
time battery_AA is charged  2.5 seconds
time battery_AAA is charged : 2.5 seconds
time battery_C is charged :  800 s
we can stop charging batterys and still on the charger 
"""

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

# === Scenario Setup ===
# Initial state: All batteries are not charged, not damaged, not discarded, and chargers are free
expected_results = []
expected_predicates = []

for battery_name, battery in batteries.items():
    # Append expected results for each battery
    expected_results.extend([False, False, False, False, True])  # Not charged, Not charging, Not damaged, Not discarded, Charger free

# === Initial State Check ===
# Run the semantic reasoning to get the initial state
answers, predicates = semantic.get_state()

# Print and verify the initial state
print("==== Initial State Check ====")
for i in range(len(answers)):
    print(f"{predicates[i]}: {answers[i]} (Expected: {expected_results[i]})")
    assert answers[i] == expected_results[i]  # Ensure the actual state matches the expected state

# === State Changes ===
expected_results = []
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

# Change 2: Charge battery_AAA until it becomes "damaged"
batt_AAA = batteries['battery_AAA/1']

sim_env.place_object_in_charger(batt_AAA, [-0.7, -0.9, 0.09])  # Place battery_AAA in its charger
print("== Starting to charge battery_AAA ===")
executor.wait(100)

sim_env.wait(11)  # Wait long enough for the battery to become damaged
# expected_results for AAA
expected_results.extend([True, False, True, True,  True])
#this for AA
expected_results.extend([True, False, False, False, False])

batt_AAA.stop_charging()  # Stop charging battery_AAA
print("== Finished charging battery_AAA ===")

print("==Remove battery_AAA from charger===")
sim_env.remove_object_from_charger(batt_AAA.name, [-0.55, 0.0, 0.03])  # Remove battery_AAA from its charger

# Change 3: Place battery_C on its charger to make its charger "not free"
print("== Starting to place battery_C in charger ===")
# print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")

sim_env.place_object_in_charger(batteries['battery_C/1'], [-0.7, -0.65, 0.09])  # Place battery_C in its charger
expected_results.extend([False, True, False, False, False])
# print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
# sim_env.wait(100)  # Wait for the simulation to update

expected_results.extend([False, False, False, False, True])

print("===============")
print("State Check After Changes")
# === Expected Results After Changes ===
# expected_results = []
# for battery_name, battery in batteries.items():
#     # Compute the expected state for each battery after the changes
#     is_charged = semantic.is_charged(battery)
#     is_damaged = semantic.is_damaged(battery)
#     should_discard = semantic.should_be_discarded(battery)
#     charger_free = semantic.is_charger_free_for(battery)
#     expected_results.extend([is_charged, is_damaged, should_discard, charger_free])

# === After Changes Check ===
# Run the semantic reasoning to get the state after changes
answers, predicates = semantic.get_state()

# Print and verify the state after changes
print("==== After Changes Check ====")
for ans, pred, exp in zip(answers, predicates, expected_results):
    print(f"{pred}: {ans} (Expected: {exp})")
    assert ans == exp  # Ensure the actual state matches the expected state

print("✅ All get_state checks passed.")
sim_env.wait(100)  # Wait for the simulation to update



