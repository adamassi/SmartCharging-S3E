# semantic_questions.py

def is_charged(battery, sim_env):
    """
    Returns True if the battery is charged.
    :param battery: Name of the battery object
    :param sim_env: Simulation environment instance
    """
    # Replace this logic with your own charge detection method
    return battery in sim_env.get_object_manager().charged_batteries


def is_compatible_with(charger: str, battery):
    """
    Returns True if the charger is compatible with the battery.
    :param charger: Name of the charger object
    :param battery: Name of the battery object
    :param sim_env: Simulation environment instance
    """
    
    
     
    charger_type = charger.split("_")[1].strip("/")
    # print(charger_type)  # Output: AA
    return charger_type == battery.battery_type
    

    # for battery_type in compatibility:
    #     if battery.startswith(battery_type):
    #         return charger in compatibility[battery_type]
    # return False


def is_damaged(battery, sim_env):
    """
    Returns True if the battery is physically damaged.
    :param battery: Name of the battery object
    :param sim_env: Simulation environment instance
    """
    return battery in sim_env.get_object_manager().damaged_batteries


def should_be_discarded(battery, sim_env):
    """
    Returns True if the battery is damaged or not compatible with any charger.
    :param battery: Name of the battery object
    :param sim_env: Simulation environment instance
    """
    if is_damaged(battery, sim_env):
        return True

    chargers = sim_env.get_object_manager().get_all_charger_names()
    return not any(is_compatible_with(charger, battery, sim_env) for charger in chargers)
