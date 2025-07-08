# semantic_questions.py

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
        return battery.is_damaged

    def should_be_discarded(self, battery):
        """
        Returns True if the battery is damaged or not compatible with any charger.
        :param battery: Battery class instance
        """
        if self.is_damaged(battery):
            return True

        chargers = self.sim_env.get_object_manager().get_all_charger_names()
        return not any(self.is_compatible_with(charger, battery) for charger in chargers)

    def is_charger_free_for(self, battery):
        """
        Returns True if the charger that matches the battery's type is currently free (no battery inside).
        :param battery: Battery class instance
        """
        self.sim_env.valid_geometry_names()
        



