import time
# how long can but battery be charged before it is considered damaged
battery_died = 20

class Battery:
    """
    Represents a battery with charging tracking and type specification.
    Tracks start/end charging times and computes charge percentage.
    """
    def __init__(self, name: str, initial_charge: float = 0.0, charging_rate: float = 1.0, discharge_rate: float = 0.1):
        """
        :param name: Unique identifier for the battery (e.g., 'battery_AAA').
        :param initial_charge: Initial charge percentage (0 to 100).
        :param charging_rate: Charging speed in percent per second.
        :param discharge_rate: Discharge speed in percent per second when not charging.
        """
        self.name = name
        self.battery_type = self._extract_battery_type(name)
        self.charge_percent = max(0.0, min(initial_charge, 100.0))
        self.charging_rate = charging_rate
        self.discharge_rate = discharge_rate
        self.creation_time = time.time()  # 🆕 Battery creation time

        self.is_charging = False
        self.charge_start_time = None
        self.charge_end_time = None
        self.is_damaged = False  
        self.last_discharge_time = time.time()  # Track the last time the battery was discharged

    def _extract_battery_type(self, name: str) -> str:
        """
        Extracts the battery type from the name.
        Example: 'battery_AAA/user_joint_1/' -> 'AAA'
        """
        # Remove everything after the first '/'
        name = name.split('/')[0]

        if "battery_" in name.lower():
            parts = name.split("_")
            if len(parts) > 1:
                return parts[1].upper()
        return "UNKNOWN"

    def start_charging(self):
        if not self.is_charging:
            self.charge_start_time = time.time()
            self.is_charging = True
            print(f"[{self.name}] ({self.battery_type}) Charging started at {self.charge_start_time}")

    def stop_charging(self):
        if self.is_charging:
            self.charge_end_time = time.time()
            self._update_charge()
            self.is_charging = False
            print(f"[{self.name}] ({self.battery_type}) Charging stopped at {self.charge_end_time}. Charge: {self.charge_percent:.2f}%")
            self.check_is_damaged()

    def _update_charge(self):
        if self.charge_start_time and self.charge_end_time:
            elapsed_time = self.charge_end_time - self.charge_start_time
            added_charge = elapsed_time * self.charging_rate
            self.charge_percent = min(100.0, self.charge_percent + added_charge)

    def check_charge_progress(self):
        """
        Check the current charge percentage while the battery is charging.
        """
        if self.is_charging:
            elapsed_time = time.time() - self.charge_start_time
            added_charge = elapsed_time * self.charging_rate
            current_charge = min(100.0, self.charge_percent + added_charge)
            print(f"[{self.name}] ({self.battery_type}) Current charge: {current_charge:.2f}%")
            return current_charge
        else:
            print(f"[{self.name}] ({self.battery_type}) Battery is not charging.")
            return self.charge_percent

    def discharge_battery(self):
        """
        Decrease the battery percentage over time when it is not charging.
        """
        if not self.is_charging:
            current_time = time.time()
            elapsed_time = current_time - self.last_discharge_time
            discharged_amount = elapsed_time * self.discharge_rate
            self.charge_percent = max(0.0, self.charge_percent - discharged_amount)
            self.last_discharge_time = current_time
            print(f"[{self.name}] ({self.battery_type}) Discharged. Current charge: {self.charge_percent:.2f}%")

    def get_status(self):
        return {
            "battery": self.name,
            "type": self.battery_type,
            "is_charging": self.is_charging,
            "charge_percent": round(self.charge_percent, 2),
            "start_time": self.charge_start_time,
            "end_time": self.charge_end_time,
            "elapsed_time": (self.charge_end_time - self.charge_start_time)
            if self.charge_start_time and self.charge_end_time else None
        }

    def check_is_damaged(self):
        """
        Check if the battery is damaged.
        This is a placeholder for actual damage detection logic.
        """
        if self.is_damaged:
            return True
        elif self.is_charging:
            if time.time() - self.charge_start_time > battery_died:
                self.is_damaged = True
                print(f"[{self.name}] ({self.battery_type}) Battery is damaged due to overcharging.")
                return True

    def __str__(self):
        return f"Battery({self.name}, Type: {self.battery_type}): {self.charge_percent:.2f}%, Charging: {self.is_charging}"

    def render_with_timer(self, sim_env):
        """
        Call the simulation environment's render_with_timer method.
        :param sim_env: Instance of the SimEnv class.
        """
        sim_env.render_with_timer()
