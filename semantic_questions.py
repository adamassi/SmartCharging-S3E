# semantic_questions.py
import numpy as np

class SmartChargingSemantic:
    def __init__(self, sim_env):
        """
        Initialize the SmartChargingSemantic class with the simulation environment.
        :param sim_env: Simulation environment instance
        """
        self.sim_env = sim_env

    def is_charged(self, battery):
        """
        Returns True if the battery is charged.
        :param battery: Battery class instance
        """
        return battery.charge_percent > 60.0

    def is_compatible_with(self, charger: str, battery):
        """
        Returns True if the charger is compatible with the battery.
        :param charger: Name of the charger object
        :param battery: Battery class instance
        """
        charger_type = charger.split("_")[1].strip("/")
        return charger_type == battery.battery_type

    def is_damaged(self, battery):
        """
        Returns True if the battery is physically damaged.
        :param battery: Battery class instance
        """
        return self.sim_env.check_is(battery.name)

    def should_be_discarded(self, battery):
        """
        Returns True if the battery is damaged .
        :param battery: Battery class instance
        """
        if self.is_damaged(battery):
            return True
        return False

    def is_charger_free_for(self, battery):
        """
        Returns True if the charger that matches the battery's type is currently free (no battery inside).
        :param battery: Battery class instance
        """
        geoms_names = self.sim_env.get_valid_geometry_names()
        # keep only names that do not contain table
        geoms_names = [geom for geom in geoms_names if "table" not in geom]
        charger_geom = f"{battery.battery_type}_charger/bottom"
        for geom in geoms_names:
            # print(geom)
            normal_force = self.sim_env.get_normal_force(charger_geom,geom)
            # print(normal_force)
            # Compare normal_force with np.array([0, 0, 0])
            if not np.array_equal(normal_force, np.array([0, 0, 0])):
                # if  normal_force[2] > 0.9:
                    print(f"Normal force on {geom}: {normal_force}")
                    return False
        return True


