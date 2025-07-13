import time
# how long can the battery be charged before it is considered damaged
battery_died = 4
# class Batteris:
#     def __init__(self,env):
#         """
#         Initialize the Batteris class with a simulation environment.
#         :param env: The simulation environment instance.
#         """
#         self.battery = {}
#         joint_names = env.get_all_joint_names()  # Get all joint names from the simulation environment
#         # Create Battery instances for joints containing "battery" in their name
#         for joint_name in joint_names:
#             if "battery" in joint_name.lower():
#                 key = joint_name.split('-')[0]
#                 batteryi = Battery(name=joint_name, env=env)
#                 self.battery[key] = batteryi  # Store the battery instance in the dictionary


class Battery:
    """
    Represents a battery with charging tracking and type specification.
    Tracks start/end charging times and computes charge percentage.
    """
    

    def __init__(self, name: str, initial_charge: float = 0.0, charging_rate: float = 40.0, discharge_rate: float = 0.1 ,env=None):
        """
        Initialize the battery object.
        :param name: Unique identifier for the battery (e.g., 'battery_AAA').
        :param initial_charge: Initial charge percentage (0 to 100).
        :param charging_rate: Charging speed in percent per second.
        :param discharge_rate: Discharge speed in percent per second when not charging.
        """
        self.name = name
        self.battery_type = self._extract_battery_type(name)
        self.charge_percent = max(0.0, min(initial_charge, 100.0))  # Ensure charge is within valid range
        self.charging_rate = charging_rate
        self.discharge_rate = discharge_rate
        self.env = env  # Reference to the simulation environment, if needed
        self.creation_time = time.time()  # Battery creation time
        # Extract the body name from the battery name
        self.body_name = name.split('/')[0]  +"/"+ name.split('/')[0]
        self.is_charging = False
        self.charge_start_time =  time.time()
        self.charge_end_time =  time.time()
        self.is_damaged = False  # Indicates if the battery is damaged
        self.last_discharge_time = time.time()  # Track the last time the battery was discharged

    def _extract_battery_type(self, name: str) -> str:
        """
        Extracts the battery type from the name.
        Example: 'battery_AAA/user_joint_1/' -> 'AAA'
        :param name: The name of the battery.
        :return: The extracted battery type.
        """
        # Remove everything after the first '/'
        name = name.split('/')[0]

        if "battery_" in name.lower():
            parts = name.split("_")
            if len(parts) > 1:
                return parts[1].upper()
        return "UNKNOWN"
# done 
    def start_charging(self):
        """
        Start charging the battery.
        """
        # self.env.select_body(self.body_name)  # Select the battery body in the simulation environment
        if not self.is_charging:
            self.charge_start_time = time.time()
            self.is_charging = True
            # print(f"[{self.name}] ({self.battery_type}) Charging started at {self.charge_start_time}")
# done
    def stop_charging(self):
        """
        Stop charging the battery and update its charge percentage.
        """
        self.env.select_body(self.body_name)
        if self.is_charging:
            self.check_is_damaged()
            self.charge_end_time = time.time()
            self._update_charge()
            self.is_charging = False
            # print(f"[{self.name}] ({self.battery_type}) Charging stopped at {self.charge_end_time}. Charge: {self.charge_percent:.2f}%")
# done
    def _update_charge(self):
        """
        Update the battery's charge percentage based on the elapsed charging time.
        """
        # self.env.select_body(self.body_name)
        if self.charge_start_time and self.charge_end_time:
            elapsed_time = self.charge_end_time - self.charge_start_time
            added_charge = elapsed_time * self.charging_rate
            # print(f"charg to add: {added_charge}")
            self.charge_percent = min(100.0, self.charge_percent + added_charge)  # Cap at 100%
# done 
    def check_charge_progress(self):
        """
        Check the current charge percentage while the battery is charging.
        :return: The current charge percentage.
        """
        
        if self.is_charging:
            elapsed_time = time.time() - self.charge_start_time
            added_charge = elapsed_time * self.charging_rate
            current_charge = min(100.0, self.charge_percent + added_charge)
            # print(f"[{self.name}] ({self.battery_type}) Current charge: {current_charge:.2f}%")
            return current_charge
        else:
            # print(f"[{self.name}] ({self.battery_type}) Battery is not charging.")
            self.discharge_battery()  # Ensure the battery is discharged if not charging
            return self.charge_percent
# done
    def discharge_battery(self):
        """
        Decrease the battery percentage over time when it is not charging.
        """
        if not self.is_charging:
            current_time = time.time()
            elapsed_time = current_time - self.last_discharge_time
            discharged_amount = elapsed_time * self.discharge_rate
            self.charge_percent = max(0.0, self.charge_percent - discharged_amount)  # Ensure charge doesn't go below 0
            self.last_discharge_time = current_time
            # print(f"[{self.name}] ({self.battery_type}) Discharged. Current charge: {self.charge_percent:.2f}%")

    def get_status(self):
        """
        Get the current status of the battery.
        :return: A dictionary containing the battery's status.
        """
        self.discharge_battery()  # Ensure the battery is discharged before getting status
        # TODO calculate battery status based on charge percentage and charging state
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
        Check if the battery is damaged due to overcharging.
        :return: True if the battery is damaged, False otherwise.
        """
        self.env.select_body(self.body_name)
        if self.is_damaged:
            return True
        elif self.is_charging:
            # print(time.time() - self.charge_start_time)
            if time.time() - self.charge_start_time > battery_died:
                # If the battery has been charging for too long, mark it as damaged
                self.is_damaged = True
                name = self.name.split('/')[0]
                self.env.change_battery_color(name+'/battery_body', [1,0,0,1])  # Change color to red to indicate damage
                # print(f"[{self.name}] ({self.battery_type}) Battery is damaged due to overcharging.")
                return True
        else:
            if self.charge_end_time - self.charge_start_time > battery_died:
                self.is_damaged = True
                name = self.name.split('/')[0]
                self.env.change_battery_color(name+'/battery_body', [1,0,0,1])
                # print(f"[{self.name}] ({self.battery_type}) Battery is damaged due to prolonged discharge.")
                return True
        return False

    def __str__(self):
        """
        String representation of the battery.
        :return: A string describing the battery's status.
        """
        return f"Battery({self.name}, Type: {self.battery_type}): {self.charge_percent:.2f}%, Charging: {self.is_charging}"

    # def render_with_timer(self, sim_env):
    #     """
    #     Call the simulation environment's render_with_timer method.
    #     :param sim_env: Instance of the SimEnv class.
    #     """
    #     sim_env.render_with_timer()
