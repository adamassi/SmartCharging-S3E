import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import numpy as np
from semantic_questions import SmartChargingSemantic
from sim_ur5.mujoco_env.sim_env import SimEnv
from sim_ur5.motion_planning.motion_executor import MotionExecutor
from battery_class import Battery

# === Setup ===
sim_env = SimEnv()
executor = MotionExecutor(sim_env)
batteries = {}
joint_names = sim_env.get_all_joint_names()
for joint_name in joint_names:
    if "battery" in joint_name.lower():
        key = joint_name.split('-')[0]
        battery = Battery(name=joint_name, env=sim_env)
        batteries[key] = battery

semantic = SmartChargingSemantic(sim_env, batteries)
executor.wait(100)

# === Scenario Setup ===
# מצב התחלתי: כל הסוללות לא טעונות ולא פגומות ואין סוללות במטענים
expected_results = []
expected_predicates = []

for battery_name, battery in batteries.items():
    expected_results.extend([False, False, False, True])  # Not charged, Not damaged, Not discarded, Charger free

# הרצה ראשונה - בדיקה לפני כל פעולה
answers, predicates = semantic.get_state()
# print getting the state
for i in range(len(answers)):
    print(f"{predicates[i]}: {answers[i]} (Expected: {expected_results[i]})")
    
print("==== Initial State Check ====")
for ans, pred, exp in zip(answers, predicates, expected_results):
    print(f"{pred}: {ans} (Expected: {exp})")
    assert ans == exp

# שינוי מצב: נטען battery_AA מספיק כדי שיהיה charged
batt_AA = batteries['battery_AA/1']
sim_env.place_object_in_charger(batt_AA, [-0.7, -0.75, 0.09])
sim_env.wait(2)  # מספיק זמן לעבור את ה-60%
batt_AA.stop_charging()

# שינוי מצב: battery_AAA נטען עד שיהיה damaged
batt_AAA = batteries['battery_AAA/1']
sim_env.place_object_in_charger(batt_AAA, [-0.7, -0.9, 0.09])
sim_env.wait(7)
batt_AAA.stop_charging()

# שינוי מצב: מניחים battery_C על המטען שלו (כדי לבדוק charger free=False)
sim_env.place_object_in_charger(batteries['battery_C/1'], [-0.7, -2.05, 0.09])

# === Expected after changes ===
expected_results = []
for battery_name, battery in batteries.items():
    is_charged = semantic.is_charged(battery)
    is_damaged = semantic.is_damaged(battery)
    should_discard = semantic.should_be_discarded(battery)
    charger_free = semantic.is_charger_free_for(battery)
    expected_results.extend([is_charged, is_damaged, should_discard, charger_free])

# בדיקה אחרי השינויים
answers, predicates = semantic.get_state()

print("==== After Changes Check ====")
for ans, pred, exp in zip(answers, predicates, expected_results):
    print(f"{pred}: {ans} (Expected: {exp})")
    assert ans == exp

print("✅ All get_state checks passed.")

