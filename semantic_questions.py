# semantic_questions.py
import numpy as np
import time

class SmartChargingSemantic:
    """
    A class to handle semantic questions related to batteries and chargers in the simulation.
    """

    def __init__(self, sim_env, batteries):
        """
        Initialize the SmartChargingSemantic class.
        :param sim_env: The simulation environment instance.
        :param batteries: A dictionary of battery objects, keyed by their names.
        """
        self.sim_env = sim_env
        self.batteries = batteries

    def is_charged(self, battery):
        """
        Check if the battery is charged.
        :param battery: Battery class instance.
        :return: True if the battery's charge percentage is greater than 60%, False otherwise.
        """
        self.sim_env.select_body(battery.body_name)  # Select the battery body in the simulation environment
        time.sleep(0.5)  # Allow time for the simulation to update
        return battery.check_charge_progress() > 60.0

    def is_compatible_with(self, charger: str, battery):
        """
        Check if the charger is compatible with the battery.
        :param charger: Name of the charger object.
        :param battery: Battery class instance.
        :return: True if the charger type matches the battery type, False otherwise.
        """
        charger_type = charger.split("_")[1].strip("/")
        return charger_type == battery.battery_type

    def has_passed_one_year(self, battery):
        """
        Check if the battery has passed one year since its creation.
        :param battery: Battery class instance.
        :return: True if the battery has passed one year, False otherwise.
        """
        # One year in seconds (365 days)
        one_year_in_seconds = 365 * 24 * 60 * 60
        elapsed_time = time.time() - battery.creation_time
        return elapsed_time >= one_year_in_seconds

    def is_damaged(self, battery):
        """
        Check if the battery is physically damaged.
        :param battery: Battery class instance.
        :return: True if the battery is damaged, False otherwise.
        """
        self.sim_env.select_body(battery.body_name)  # Select the battery body in the simulation environment
        return battery.check_is_damaged()

    def should_be_discarded(self, battery):
        """
        Check if the battery should be discarded.
        :param battery: Battery class instance.
        :return: True if the battery is damaged or has passed one year, False otherwise.
        """
        return self.is_damaged(battery) or self.has_passed_one_year(battery)

    def is_charger_free_for(self, battery):
        """
        Check if the charger for the given battery type is free (not occupied by another battery).
        :param battery: Battery class instance.
        :return: True if the charger is free or battery is charging, False otherwise.
        """
        type_charger = battery.battery_type + "_charger/charger_" + battery.battery_type  # Define the charger type based on the battery type
        self.sim_env.select_body(type_charger)  # Select the battery body in the simulation environment

        # Get all valid geometry names in the simulation
        geoms_names = self.sim_env.get_valid_geometry_names()

        # Filter geometries that start with 'battery_'
        geoms_names = [geom for geom in geoms_names if geom.startswith('battery_')]

        # Define the charger geometry name for the battery type
        charger_geom = f"{battery.battery_type}_charger/bottom"

        # Check if any geometry is applying a force on the charger
        for geom in geoms_names:
            normal_force = self.sim_env.get_normal_force(charger_geom, geom)
            if not np.array_equal(normal_force, np.array([0, 0, 0])):
                return False  # Charger is occupied
        return True  # Charger is free

    def is_charging(self, battery):
        """
        Check if the battery is currently charging.
        :param battery: Battery class instance.
        :return: True if the battery is charging, False otherwise.
        """
        return battery.is_charging

    def get_state(self):
        """
        Get the semantic state of all batteries in the simulation.
        :return: A tuple containing:
                 - A NumPy array of boolean answers to semantic questions.
                 - A list of predicates describing the semantic questions.
        """
        answers = []  # List to store answers to semantic questions
        predicates = []  # List to store predicates for semantic questions

        for battery_name, battery in self.batteries.items():
            # Semantic Question 1: Is the battery charged?
            charged = self.is_charged(battery)
            answers.append(charged)
            predicates.append(f"IsCharged({battery.name})")

            # Semantic Question 2: Is the battery charging?
            charging = self.is_charging(battery)
            answers.append(charging)
            predicates.append(f"IsCharging({battery.name})")

            # Semantic Question 3: Is the battery damaged?
            damaged = self.is_damaged(battery)
            answers.append(damaged)
            predicates.append(f"IsDamaged({battery.name})")

            # Semantic Question 4: Should the battery be discarded?
            discarded = self.should_be_discarded(battery)
            answers.append(discarded)
            predicates.append(f"ShouldBeDiscarded({battery.name})")

            # Semantic Question 5: Is the charger free for the battery?
            charger_free = self.is_charger_free_for(battery)
            answers.append(charger_free)
            predicates.append(f"IsChargerFreeFor({battery.name})")

        # Return the answers as a NumPy array and the predicates as a list
        return np.array(answers), predicates

    def success_score(self, current_state, goal):
        """
        Calculates a success score based on the state of individual batteries.
        The total score is divided among batteries defined in the goal.
        A battery gets its points if it meets the 'IsCharged' goal. 80 if the battery is charging, 60 percent if the charger is free and the battery is not charged, 50 percent if the charger is not free and it is not charged, 20 percent if the battery is not charged and there is a battery of the same type in the charger, but gets 0 if it's damaged.

        :param current_state: A tuple (answers, predicates) from semantic.get_state().
        :param goal: A list of goal predicates.
        :return: A success score from 0 to 100.
        """
        current_answers, current_predicates = current_state
        current_results = dict(zip(current_predicates, current_answers))
        total_score = 0.0
        # print(current_results)
        for predicate in goal:
            if predicate.startswith("IsCharged"):
                battery_name = predicate.split("(")[1].split(")")[0]
                if current_results.get(f"ShouldBeDiscarded({battery_name})", True):
                    total_score += 0.0
                elif current_results.get(f"IsCharged({battery_name})", True):
                    total_score += 100.0
                elif current_results.get(f"IsCharging({battery_name})", False):
                    total_score += 80.0
                elif current_results.get(f"IsChargerFreeFor({battery_name})", False):
                    total_score += 60.0
                elif not current_results.get(f"IsChargerFreeFor({battery_name})", True) and not current_results.get(f"IsCharged({battery_name})", False):
                    total_score += 50.0
                elif not current_results.get(f"IsCharged({battery_name})", False) and current_results.get(f"IsChargerFreeFor({battery_name})", True):
                    total_score += 20.0
                else:
                    total_score += 0.0  # Battery is damaged or does not meet any criteria

        # Divide the total score by the number of elements in the goal
        if len(goal) > 0:
            total_score /= len(goal)

        return total_score

