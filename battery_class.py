import time


class Battery:
    """
    Represents a battery with charging tracking and type specification.
    Tracks start/end charging times and computes charge percentage.
    """
    def __init__(self, name: str, initial_charge: float = 0.0, charging_rate: float = 1.0):
        """
        :param name: Unique identifier for the battery (e.g., 'battery_AAA').
        :param initial_charge: Initial charge percentage (0 to 100).
        :param charging_rate: Charging speed in percent per second.
        """
        self.name = name
        self.battery_type = self._extract_battery_type(name)
        self.charge_percent = max(0.0, min(initial_charge, 100.0))
        self.charging_rate = charging_rate

        self.is_charging = False
        self.charge_start_time = None
        self.charge_end_time = None

    def _extract_battery_type(self, name: str) -> str:
        """
        Extracts the battery type from the name.
        Example: 'battery_AAA' -> 'AAA'
        """
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

    def _update_charge(self):
        if self.charge_start_time and self.charge_end_time:
            elapsed_time = self.charge_end_time - self.charge_start_time
            added_charge = elapsed_time * self.charging_rate
            self.charge_percent = min(100.0, self.charge_percent + added_charge)

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

    def __str__(self):
        return f"Battery({self.name}, Type: {self.battery_type}): {self.charge_percent:.2f}%, Charging: {self.is_charging}"
