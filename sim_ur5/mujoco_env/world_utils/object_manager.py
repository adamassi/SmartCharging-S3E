import random
import numpy as np
from typing import Dict, List


class ObjectManager:
    """convenience class to manage graspable objects in the mujoco simulation"""

    def __init__(self, mj_model, mj_data):
        self._mj_model = mj_model
        self._mj_data = mj_data

        # manipulated objects have 6dof free joint that must be named in the mcjf.
        all_joint_names = [self._mj_model.joint(i).name for i in range(self._mj_model.njnt)]

        # all bodies that ends with "box"
        self.object_names = [name for name in all_joint_names if name.startswith("battery")]
        print(f"ObjectManager: found {len(self.object_names)} objects: {self.object_names}")
        self.objects_mjdata_dict = {name: self._mj_model.joint(name) for name in self.object_names}
        self.initial_positions_dict = self.get_all_battery_positions()
        self.workspace_x_lims = [-0.9, -0.54]
        self.workspace_y_lims = [-1.0, -0.55]
        self.battery_size = .02

    def reset(self, randomize=True, battery_positions=None):
        """
        Reset the object positions in the simulation.
        Args:
            randomize: if True, randomize the positions of the batterys, otherwise set them to initial positions.
        """
        # print("resetting object positionsAAAAAAAAAAAAAAAAAAAAAAAAA")
        # orint the battery positions
        # print(f"battery_positions: {battery_positions}")
        def check_battery_collision(new_pos):
            """Tests if new position for battery collides with any other battery"""
            for pos in battery_positions:
                # print("checking collisionBBBBBBBBBBBBBBBBBBB")
                # print(f"new_pos: {new_pos}, pos: {pos}")
                pos_np = np.array(pos)
                if np.linalg.norm(new_pos - pos_np) < 2 * self.battery_size:
                    return True
            battery_positions.append(list(new_pos))
            return False

        if randomize:
            # print("randomizing battery positionsCCCCCCCCCCCCCCCCC")
            # randomize battery positions
            battery_positions = []
            for _ in range(len(self.object_names)):
                # generate random position for battery
                battery_location = [random.uniform(*self.workspace_x_lims), random.uniform(*self.workspace_y_lims), 0.05]
                # check if battery collides with any other previous new battery position
                while check_battery_collision(np.array(battery_location)):
                    # generate new random position for battery
                    battery_location = [random.uniform(*self.workspace_x_lims), random.uniform(*self.workspace_y_lims),
                                      0.05]
            # set batterys to new positions
            self.set_all_battery_positions(battery_positions)
        else:
            if battery_positions:
                self.set_all_battery_positions(battery_positions)
            else:
                self.set_all_battery_positions(list(self.initial_positions_dict.values()))

    def get_object_pos(self, name: str):
        return self._mj_data.joint(name).qpos[:3]

    def set_object_pose(self, name: str, pos, quat):
        joint_id = self.objects_mjdata_dict[name].id
        pos_adr = self._mj_model.jnt_qposadr[joint_id]
        self._mj_data.qpos[pos_adr:pos_adr + 7] = np.concatenate([pos, quat])

    def set_object_vel(self, name: str, cvel):
        joint_id = self.objects_mjdata_dict[name].id
        vel_adr = self._mj_model.jnt_dofadr[joint_id]
        self._mj_data.qvel[vel_adr:vel_adr + 6] = cvel

    def get_battery_position_from_mj_id(self, battery_id: int) -> np.ndarray:
        """
        Get the position of a battery in the simulation.
        Args:
            battery_id: the id of the battery to get the position of.
        Returns:
            the position of the battery in format [x, y ,z].
        """
        return self._mj_data.joint(battery_id).qpos[:3]

    def get_all_battery_positions_dict(self) -> Dict[str, np.ndarray]:
        """
        Get the positions of all batterys in the simulation.
        Returns:
            a dictionary of battery names to their positions, positions will be in format {name: [x, y ,z], ...}.
        """
        return {name: self.get_battery_position_from_mj_id(self.objects_mjdata_dict[name].id) for name in self.object_names}

    def get_all_battery_positions(self) -> List[np.ndarray]:
        """
        Get the positions of all batterys in the simulation.
        Returns:
            a dictionary of battery names to their positions, positions will be in format {name: [x, y ,z], ...}.
        """
        return [self.get_battery_position_from_mj_id(self.objects_mjdata_dict[name].id) for name in self.object_names]

    def set_battery_position(self, battery_id, position):
        """
        Set the position of a battery in the simulation.
        Args:
            battery_id: the id of the battery to set the position of.
            position: the position to set the battery to, position will be in format [x, y ,z].
        """
        # joint_name = f"battery{battery_id}_fj"
        # joint_id = self._mj_model.joint(joint_name).id
        # pos_adrr = self._mj_model.jnt_qposadr[joint_id]
        # self._mj_data.qpos[pos_adrr:pos_adrr + 3] = position

    def set_all_battery_positions(self, positions):
        """
        Set the positions of all batterys in the simulation.
        Args:
            positions: a list of positions to set the batterys to, positions will be in format [[x, y ,z], ...].
        """
        # set batterys positions
        for i, pos in enumerate(positions):
            self.set_battery_position(i, pos)

    # def rename_object(self, old_name: str, new_name: str):
    #     """
    #   Rename an object in the simulation.not implemented yet
    #     Rename an object in the simulation.
    #     Args:
    #         old_name: The current name of the object.
    #         new_name: The new name for the object.
    #     """
    #     if old_name not in self.object_names:
    #         raise ValueError(f"Object '{old_name}' does not exist in the simulation.")

    #     # Update the name in the object_names list
    #     self.object_names.remove(old_name)
    #     self.object_names.append(new_name)

    #     # Update the key in the objects_mjdata_dict
    #     self.objects_mjdata_dict[new_name] = self.objects_mjdata_dict.pop(old_name)

    #     # Update the name in the MuJoCo model (if applicable)
    #     joint = self._mj_model.joint(old_name)
    #     print(joint)
    #     joint.name = new_name

    #     print(f"✅ Renamed object '{old_name}' to '{new_name}'.")
