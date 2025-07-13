# semantic_questions.py
import numpy as np

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
# done 
    def is_charged(self, battery):
        """
        Check if the battery is charged.
        :param battery: Battery class instance.
        :return: True if the battery's charge percentage is greater than 60%, False otherwise.
        """
        self.sim_env.select_body(battery.body_name)  # Select the battery body in the simulation environment
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
    

# DONE 
    def is_damaged(self, battery):
        """
        Check if the battery is physically damaged.
        :param battery: Battery class instance.
        :return: True if the battery is damaged, False otherwise.
        """
        return battery.check_is_damaged()
# done 
    def should_be_discarded(self, battery):
        """
        Check if the battery should be discarded.
        :param battery: Battery class instance.
        :return: True if the battery is damaged, False otherwise.
        """
        return self.is_damaged(battery)

    def is_charger_free_for(self, battery):
        """
        Check if the charger for the given battery type is free (not occupied by another battery).
        :param battery: Battery class instance.
        :return: True if the charger is free, False otherwise.
        """
        # Get all valid geometry names in the simulation
        geoms_names = self.sim_env.get_valid_geometry_names()
        # print(geoms_names)
        # keep only geometries that are starting with the battery 
        geoms_names = [geom for geom in geoms_names if geom.startswith('battery_')]

        # Exclude geometries related to the table
        # geoms_names = [geom for geom in geoms_names if "table" not in geom]

        # Define the charger geometry name for the battery type
        charger_geom = f"{battery.battery_type}_charger/bottom"

        # Check if any geometry is applying a force on the charger
        for geom in geoms_names:
            normal_force = self.sim_env.get_normal_force(charger_geom, geom)
            if not np.array_equal(normal_force, np.array([0, 0, 0])):
                return False  # Charger is occupied
        return True  # Charger is free

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

            # Semantic Question 2: Is the battery damaged?
            damaged = self.is_damaged(battery)
            answers.append(damaged)
            predicates.append(f"IsDamaged({battery.name})")

            # Semantic Question 3: Should the battery be discarded?
            discarded = self.should_be_discarded(battery)
            answers.append(discarded)
            predicates.append(f"ShouldBeDiscarded({battery.name})")

            # Semantic Question 4: Is the charger free for the battery?
            charger_free = self.is_charger_free_for(battery)
            answers.append(charger_free)
            predicates.append(f"IsChargerFreeFor({battery.name})")

        # Return the answers as a NumPy array and the predicates as a list
        return np.array(answers), predicates


