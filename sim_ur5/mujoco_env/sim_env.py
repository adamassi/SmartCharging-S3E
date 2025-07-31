""" a wrapper around spear env to simplify and fix some issues with the environment """
from copy import deepcopy, copy
from collections import namedtuple
import mujoco as mj
from mujoco import MjvCamera, mjtFont
# from mujoco.mjbindings.functions import mjv_addText  # Commented out as it cannot be resolved
from sim_ur5.mujoco_env import MujocoEnv
from sim_ur5.mujoco_env.world_utils.object_manager import ObjectManager
from sim_ur5.mujoco_env.world_utils.grasp_manager import GraspManager
from sim_ur5.mujoco_env.world_utils.configurations_and_constants import *
from sim_ur5.utils.logging_util import setup_logging
from scipy.spatial.transform import Rotation as R
import time

from sim_ur5.mujoco_env.rendering import WindowRenderer


class SimEnv:
    def __init__(self, render_mode='human', cfg=muj_env_config, render_sleep_to_maintain_fps=True):
        self.render_mode = render_mode
        self._env = MujocoEnv.from_cfg(cfg=cfg, render_mode=render_mode, frame_skip=frame_skip,
                                       sleep_to_maintain_fps=render_sleep_to_maintain_fps)
        self.frame_skip = frame_skip
        obs, info = self._env.reset()  # once, for info, later again
        self._mj_model = info['privileged']['model']
        self._mj_data = info['privileged']['data']
        self._env_entities = {name: agent.entity for name, agent in self._env.agents.items()}
        self.robots_joint_pos = {}
        self.robots_joint_velocities = {}
        self.robots_force = {}
        self.object_timers ={}
        self.robots_camera = {}
        for agent in self._env_entities.keys():
            self.robots_joint_pos[agent] = np.zeros((1, 6))  # will be updated in reset
            self.robots_joint_velocities[agent] = np.zeros((1, 6))  # --""--
            self.robots_force[agent] = 0.0
            self.robots_camera[agent] = []
        self.gripper_state_closed = False  # --""--
        self.max_joint_velocities = INIT_MAX_VELOCITY

        self._object_manager = ObjectManager(self._mj_model, self._mj_data)
        self._grasp_manager = GraspManager(self._mj_model, self._mj_data, self._object_manager, min_grasp_distance=0.1)

        self.num_batterys = len(self._object_manager.object_names)

        self.image_res_h = 720
        self.image_res_w = 1280
        self.renderer = mj.Renderer(self._mj_model, self.image_res_h, self.image_res_w)
        self._mj_model.camera("robot-cam").fovy[0] = 45

        self._ee_mj_data = self._mj_data.body('robot_0_ur5e/robot_0_adhesive gripper/')
        #self._ee_mj_data = self._mj_data.body('rethink_mount_stationary/robot_0_ur5e/robot_0_adhesive gripper/')
        # self.dt = self._mj_model.opt.timestep * frame_skip
        # self._pid_controller = PIDController(kp, ki, kd, dt)

        setup_logging()

    def close(self):
        self._env.close()

    def set_robot_joints(self, robot_name, joint_pos, joint_vel=(0,) * 6, simulate_step=True):
        self._env_entities[robot_name].set_state(position=joint_pos, velocity=joint_vel)
        if simulate_step:
            self.simulate_steps(1)

    def set_battery_positions_on_table(self, battery_positions_xy):
        z = 0.03
        self._object_manager.set_all_battery_positions([[x, y, z] for x, y in battery_positions_xy])

    def reset(self, randomize=True, battery_positions=None):
        self.max_joint_velocities = INIT_MAX_VELOCITY

        obs, _ = self._env.reset()
        agents = obs.keys()

        for agent in agents:
            self.set_robot_joints(agent, [-np.pi / 2, -np.pi / 2, 0, -np.pi / 2, 0, 0], simulate_step=False)

        for agent in agents:
            self.robots_joint_pos[agent] = obs[agent]['robot_state'][:6]
            self.robots_joint_velocities[agent] = obs[agent]["robot_state"][6:12]
            # self.robots_force[agent] = obs[agent]['sensor']
            self.robots_camera[agent] = [obs[agent]['camera'], obs[agent]['camera_pose']]
        self.gripper_state_closed = False
        self._grasp_manager.release_object()
        self._object_manager.reset(randomize=randomize, battery_positions=battery_positions)################################

        self.step(self.robots_joint_pos, gripper_closed=False)

        if self.render_mode == "human":
            self._env.render()

        return self.get_state()

    def step(self, target_joint_pos, gripper_closed=None):
        # if reset_pid:
        #     self._pid_controller.reset_endpoint(target_joint_pos)
        if gripper_closed is None:
            gripper_closed = self.gripper_state_closed
        self.gripper_state_closed = gripper_closed

        self._env_step(target_joint_pos)
        self._clip_joint_velocities()

        if gripper_closed:
            if self._grasp_manager.attached_object_name is not None:
                self._grasp_manager.update_grasped_object_pose()
            else:
                self._grasp_manager.grasp_battery_if_close_enough()
        else:
            self._grasp_manager.release_object()

        if self.render_mode == "human":
            self._env.render()

        return self.get_state()

    def simulate_steps(self, n_steps):
        """
        simulate n_steps in the environment without moving the robot
        """
        config = self.robots_joint_pos
        for _ in range(n_steps):
            self.step(config)

    def render(self):
        if self.render_mode == "human":
            return self._env.render()
        return None

    def get_state(self):
        # object_positions = self._object_manager.get_all_battery_positions_dict()
        state = {"robots_joint_pos": self.robots_joint_pos,
                 "robots_joint_velocities": self.robots_joint_velocities,
                 # "robots_force": self.robots_force,
                 # "robots_camera": self.robots_camera,
                 "gripper_state_closed": self.gripper_state_closed,
                 # "object_positions": object_positions,
                 "grasped_object": self._grasp_manager.attached_object_name,
                 "geom_contact": convert_mj_struct_to_namedtuple(self._env.sim.data.contact)}

        return deepcopy(state)

    def get_tower_height_at_point(self, point):
        battery_positions = self._object_manager.get_all_battery_positions_dict()
        not_grasped_battery_positions = {name: pos for name, pos in battery_positions.items()
                                       if name != self._grasp_manager.attached_object_name}
        not_grasped_battery_positions = np.array(list(not_grasped_battery_positions.values()))

        batterys_near_point = not_grasped_battery_positions[
            np.linalg.norm(not_grasped_battery_positions[:, :2] - point, axis=1) < 0.03]
        highest_battery_height = np.max(batterys_near_point[:, 2]) if batterys_near_point.size > 0 \
            else not_grasped_battery_positions[:, 2].min() - 0.02
        # if no batterys near point, return zero height which is estimated as lowest battery minus battery size
        return copy(highest_battery_height)

    def get_battery_positions(self):
        return list(self._object_manager.get_all_battery_positions_dict().values())

    def set_gripper(self, closed: bool):
        """
        close/open gripper and don't change robot configuration
        @param closed: true if gripper should be closed, false otherwise
        @return: None
        """
        self.step(self.robots_joint_pos, closed)

    def _clip_joint_velocities(self):
        # new_vel = self.robots_joint_velocities.copy()
        # for agent, vel in new_vel.items():
        #     new_vel[agent] = np.clip(vel, -self.max_joint_velocities, self.max_joint_velocities)
        #     self._env_entities[agent].set_state(velocity=new_vel[agent])
        # self.robots_joint_velocities = new_vel
        return

    def _env_step(self, target_joint_pos):
        """ run environment step and update state of self accordingly"""

        # joint_control = self._pid_controller.control(self.robot_joint_pos)  # would be relevant if we change to force
        # control and use PID controller, but we are using position control right now.

        # gripper control for the environment, which is the last element in the control vector is completely
        # ignored right now, instead we attach the nearest graspable object to the end effector and maintain it
        # with the grasp manager, outside the scope of this method.

        # action = np.concatenate((target_joint_pos, [int(gripper_closed)])) # would have been if grasping worked
        actions = {}
        for agent, action in target_joint_pos.items():
            actions[agent] = np.concatenate((action, [0]))

        obs, r, term, trunc, info = self._env.step(actions)
        for agent, ob in obs.items():
            self.robots_joint_pos[agent] = ob['robot_state'][:6]
            self.robots_joint_velocities[agent] = ob['robot_state'][6:12]
            # self.robots_force[agent] = obs[agent]['sensor']
            self.robots_camera[agent] = [obs[agent]['camera'], obs[agent]['camera_pose']]

    def get_ee_pos(self):
        return deepcopy(self._ee_mj_data.xpos)

    def render_image_from_pose(self, position, rotation_matrix):

        cam = self._mj_data.camera("robot-cam")
        cam.xpos = position
        cam.xmat = rotation_matrix.T.flatten()

        self.renderer.update_scene(self._mj_data, "robot-cam")

        return self.renderer.render()

    def get_robot_cam_intrinsic_matrix(self):
        cam_model = self._mj_model.camera("robot-cam")
        fovy = cam_model.fovy[0]
        res_x = self.image_res_w
        res_y = self.image_res_h

        # Convert fovy from degrees to radians
        fovy_rad = np.deg2rad(fovy)

        # Calculate focal length
        f = res_y / (2 * np.tan(fovy_rad / 2))

        # Calculate principal point
        cx = res_x / 2
        cy = res_y / 2

        # Create intrinsic matrix
        intrinsic_matrix = np.array([
            [f, 0, cx],
            [0, f, cy],
            [0, 0, 1]
        ])


        return intrinsic_matrix

    def is_object_grasped(self):
        return self._grasp_manager.attached_object_name is not None

    def get_agent_joint(self, agent_name):
        return self.robots_joint_pos[agent_name]
    def update_object_position(self, object_name, new_position):
        """
        Update the position of a specific object and ensure it affects other objects in the simulation.

        Args:
            object_name: The name of the object to update (e.g., "plate").
            new_position: A list [x, y, z] specifying the new position of the object.
        """
        # Get the object's current position
        
        
        # Update the object's position
        joint_id = self._mj_model.joint(object_name).id
        pos_adrr = self._mj_model.jnt_qposadr[joint_id]
        # print(f"Current position of {object_name}: {self._mj_data.qpos[pos_adrr:pos_adrr + 3]}")
        self._mj_data.qpos[pos_adrr:pos_adrr + 3] = new_position
        # print(f"Updated position of {object_name}: {new_position}")

        # Step the simulation to apply the changes
        self.simulate_steps(10)
        # print(f"Simulation updated with new position for {object_name}.")
        # to but in charger base_rot=[0, 1.57079632679, 0])
    def get_contacts(self, geom1_name, geom2_name):
        """
        Get the indices in the MuJoCo contacts database that match those of the given geoms, 
        and the direction of contact (1 - geom1 to geom2; -1 - geom2 to geom1).

        Args:
            geom1_name: The name of the first geometry (string).
            geom2_name: The name of the second geometry (string).

        Returns:
            contact_idx: A list of indices in the MuJoCo contacts database.
            contact_dir: A list of directions for each contact (1 or -1).
        """
        # Retrieve geometry IDs using their names
        geom1_id = mj.mj_name2id(self._mj_model, mj.mjtObj.mjOBJ_GEOM, geom1_name)
        geom2_id = mj.mj_name2id(self._mj_model, mj.mjtObj.mjOBJ_GEOM, geom2_name)

        state = self.get_state()
        contact_idx = []
        contact_dir = []

        # Iterate through all contacts in the state
        for i, (contact_id1, contact_id2) in enumerate(state['geom_contact'].geom):
            if contact_id1 == geom1_id and contact_id2 == geom2_id:
                contact_idx.append(i)
                contact_dir.append(1)
            elif contact_id2 == geom1_id and contact_id1 == geom2_id:
                contact_idx.append(i)
                contact_dir.append(-1)

        return contact_idx, contact_dir
    def get_force_on_geom(self, geom_name):
        """
        **better not use or to delete** did not work as expected
        Get the average force applied on a specific geometry.

        Args:
            geom_name: The name of the geometry (string).

        Returns:
            A numpy array representing the average force applied on the geometry.
        """
        # Ensure the geom_name exists in the environment
        geom_names = [mj.mj_id2name(self._mj_model, mj.mjtObj.mjOBJ_GEOM, geom_id) for geom_id in range(self._mj_model.ngeom)]
        if geom_name not in geom_names:
            raise ValueError(f"Geometry '{geom_name}' does not exist in the model.")

        # Get the geometry ID
        geom_id = mj.mj_name2id(self._mj_model, mj.mjtObj.mjOBJ_GEOM, geom_name)

        # Get the state and contact data
        state = self.get_state()
        contacts = state['geom_contact']

        # Collect forces for the specified geometry
        contact_forces = []
        for contact in contacts:
            # print(f'Contact: {contact}')
            # Access geom1 and geom2 using indices or field names
            geom1_id = contact[0]  # Assuming geom1 is the first field
            geom2_id = contact[1]  # Assuming geom2 is the second field
            # print(f"Contact between geom1: {geom1_id} and geom2: {geom2_id}")
            # Use .any() to handle array comparisons
            if np.any(geom1_id == geom_id) or np.any(geom2_id == geom_id):
                contact_forces.append(contact[2])  # Assuming force is the third field

        # If no forces are found, return zero
        if not contact_forces:
            return np.array([0, 0, 0])

        # Average all contact forces for one definite force
        return np.mean(contact_forces, axis=0)

    def get_normal_force(self, geom1, geom2):
        """
        Get the normal force applied by geom1 on geom2.

        Args:
            geom1: The first geometry (either a string or a geometry object with a 'name' attribute).
            geom2: The second geometry (either a string or a geometry object with a 'name' attribute).

        Returns:
            A numpy array representing the average normal force applied by geom1 on geom2.
        """
        # Ensure the geom1 and geom2 exist in the environment
        geom_names = [mj.mj_id2name(self._mj_model, mj.mjtObj.mjOBJ_GEOM, geom_id) for geom_id in range(self._mj_model.ngeom)]
        # print(geom_names)

        if geom1 not in geom_names or geom2 not in geom_names:
            raise ValueError(f"Geometry '{geom1}' OR '{geom2}' does not exist in the model.")
        # self.update_object_position("dish1_fj", [0.5, 0, 1])
        self.simulate_steps(10)

        # Get contacts and directions
        geom_contacts, geom_contact_dirs = self.get_contacts(geom1, geom2)

        if not geom_contacts:
            return np.array([0, 0, 0])

        state = self.get_state()
        frame = state['geom_contact'].frame[geom_contacts]


        # Normal force is index 0-2 of the force frame (Z-axis). Direction is always geom1 to geom2.
        # See https://github.com/google-deepmind/mujoco/blob/main/include/mujoco/mjdata.h
        all_contact_normals = frame.T[0:3]  # Transpose to enable referencing (x, y, z) at the top level
        all_contact_normals = all_contact_normals * geom_contact_dirs  # Set direction according to args order

        # Average all contact normals for one definite normal force
        return all_contact_normals.mean(axis=1)
    def valid_geometry_names(self):
        """
        Print all valid geometry names in the MuJoCo model.
        """
        state = self.get_state()
        frame = state['geom_contact']

        """ 
        state = env.get_state()
        geom1 = env._mj_model.geom('')  # Replace
        """
        print("Valid geometry names:")
        model = self._mj_model
        for geom_id in range(model.ngeom):
            geom_name = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, geom_id)
            geom_pos = self._mj_model.geom(geom_name).pos
            print(f"Geom ID {geom_id}: {geom_name},\n Position: {geom_pos}")
        # for geom in self._mj_model:
        #     print(geom)
    def get_valid_geometry_names(self):
        """ 
        Return a list of all valid geometry names in the MuJoCo model.
        """
        return [mj.mj_id2name(self._mj_model, mj.mjtObj.mjOBJ_GEOM, geom_id) for geom_id in range(self._mj_model.ngeom)]
    def is_stable_orientation(self, object_name: str, tolerance: float = 0.1) -> bool:
        """
        Checks if an object's orientation is stable based on its alignment with the upright Z-axis.

        Args:
            object_name: The name of the object to check.
            tolerance: The allowable deviation from the upright orientation (default is 0.1 radians).

        Returns:
            True if the object's orientation is stable, False otherwise.
        """
        # Get the object's orientation as a quaternion
        rotation_matrix = self._mj_data.body(object_name).xmat.reshape(3, 3)
    
        # Local Z-axis in world frame
        local_z = rotation_matrix[:, 2]
        
        # Angle between local Z and world Z
        cos_theta = np.dot(local_z, np.array([0, 0, 1]))
        angle_rad = np.arccos(np.clip(cos_theta, -1.0, 1.0))
        angle_deg = np.degrees(angle_rad)
        print(f"Angle between local Z and world Z for {object_name}: {angle_deg} degrees")
        return angle_deg < tolerance
    def place_object_in_charger(self, object, new_position):
        """
        Update the position and rotation of an object in the charger to make it upright.

        Args:
            object_name: The name of the object to update.
            new_position: A list [x, y, z] specifying the new position.
        """
        self.select_body(object.body_name)  # Select the object in the simulation
        new_rotation_euler = [0, 1.57079632679, 0]
        
       # Get the object's joint ID and qpos address
        joint_id = self._mj_model.joint(object.name).id
        pos_adrr = self._mj_model.jnt_qposadr[joint_id]

        # Update position
        self._mj_data.qpos[pos_adrr:pos_adrr + 3] = new_position

        # Update rotation if provided
        if new_rotation_euler is not None:
            # Convert Euler angles to quaternion
            new_rotation_quat = R.from_euler('xyz', new_rotation_euler).as_quat()
            rot_adrr = pos_adrr + 3  # Rotation values (quaternion) start after position (x,y,z)
            self._mj_data.qpos[rot_adrr:rot_adrr + 4] = new_rotation_quat
        
        # self.simulate_steps(1)
        # time.sleep(3)
        # Step the simulation to apply the changes
        self.simulate_steps(10)
        object.start_charging()  # Start charging the battery

    
        # time.sleep(2)
    def remove_object_from_charger(self, object_name,new_position=[0, -0.9, 0.08]):
        """
        Remove an object from the charger by resetting its position and rotation.

        Args:
            object_name: The name of the object to remove.
        """
        new_rotation_euler = [0, 0, 1.57079632679]
        
        # Convert Euler angles to quaternion
        new_rotation_quat = R.from_euler('xyz', new_rotation_euler).as_quat()

        # Get the object's joint ID and position address
        joint_id = self._mj_model.joint(object_name).id
        pos_adrr = self._mj_model.jnt_qposadr[joint_id]
        # Update the object's rotation
        rot_adrr = pos_adrr + 3  # Rotation values (quaternion)
        self._mj_data.qpos[rot_adrr:rot_adrr + 4] = new_rotation_quat
        # Update the object's position
        self._mj_data.qpos[pos_adrr:pos_adrr + 3] = new_position

        # Reset position to origin for simplicity
        # self._mj_data.qpos[pos_adrr:pos_adrr + 3] = [0, -0.9, 0.8]
    def print_all_joint_names(self):
        """
        Print all joint names in the MuJoCo model.
        """
        print("Valid joint names:")
        for joint_id in range(self._mj_model.njnt):
            joint_name = mj.mj_id2name(self._mj_model, mj.mjtObj.mjOBJ_JOINT, joint_id)
            print(f"Joint ID {joint_id}: {joint_name}")
    def get_all_joint_names(self):
        """
        Return a list of all joint names in the MuJoCo model.
        """
        return [mj.mj_id2name(self._mj_model, mj.mjtObj.mjOBJ_JOINT, joint_id) for joint_id in range(self._mj_model.njnt)]
    
    def add_object_and_reset(self, object_name, base_pos, base_rot=None):
        """
        **not tested yet**
        Add an object to the scene spec and reinitialize the simulation.
        NOTE: This recreates the whole environment.
        """
        # Ensure env is properly closed if it exists
        if hasattr(self, '_env') and self._env is not None:
            self.close()

        # Make a copy of the default scene
        new_scene_objects = list(scene.objects)

        # Define the new object
        new_object = ObjectSpec(
            object_name,
            base_pos=base_pos,
            base_joints=(JointSpec('free'),),
            base_rot=base_rot or [-1, -1, 0]
        )

        # Append new object
        new_scene_objects.append(new_object)

        # Create new scene spec
        new_scene = scene.__class__(
            scene.resource,
            objects=tuple(new_scene_objects),
            render_camera=scene.render_camera,
            init_keyframe=scene.init_keyframe
        )

        # Create new config based on muj_env_config
        new_cfg = deepcopy(muj_env_config)
        new_cfg['scene'] = new_scene

        # Reinitialize the environment
        self.__init__(cfg=new_cfg, render_mode=self.render_mode)

        print(f"✅ Added object {object_name} at position {base_pos} and reset simulation.")

    def change_battery_color(self, battery_geom_name, rgba):
        """
        Change the color of a battery's body.

        Args:
            battery_geom_name (str): The name of the geometry representing the battery body, e.g., 'battery_body'.
            rgba (list or tuple): New RGBA color values, e.g., [1, 0, 0, 1] for solid red.
        """
        
        # Find the geometry ID for the given geom name
        geom_id = mj.mj_name2id(self._mj_model, mj.mjtObj.mjOBJ_GEOM, battery_geom_name)
        if geom_id == -1:
            raise ValueError(f"Geometry '{battery_geom_name}' not found in the model.")

        # Set the RGBA color
        self._mj_model.geom_rgba[geom_id] = rgba

        # Force a render update if in human render mode
        if self.render_mode == "human":
            self._env.render()
    def wait(self, seconds):
        """
        Wait for a specified number of seconds.
        
        Args:
            seconds (float): The number of seconds to wait.
        """
        print(f"Waiting for {seconds} seconds...")
        start_time = time.time()
        while time.time() - start_time < seconds:
                pass  # wait for the specified seconds to let the simulation start
    
    def update_object_position_and_rotation(self, object_name, new_position, new_rotation_euler=[0,  1.57079632679,0]):
        """
        Update the position and optionally the rotation of a specific object.

        Args:
            object_name: The name of the object's joint to update (e.g., "plate/").
            new_position: A list [x, y, z] specifying the new position.
            new_rotation_euler: An optional list [roll, pitch, yaw] in radians for the new orientation.
                                If None, the rotation is not changed.
        """
        
        # Get the object's joint ID and qpos address
        joint_id = self._mj_model.joint(object_name).id
        pos_adrr = self._mj_model.jnt_qposadr[joint_id]

        # Update position
        self._mj_data.qpos[pos_adrr:pos_adrr + 3] = new_position

        # Update rotation if provided
        if new_rotation_euler is not None:
            # Convert Euler angles to quaternion
            new_rotation_quat = R.from_euler('xyz', new_rotation_euler).as_quat()
            rot_adrr = pos_adrr + 3  # Rotation values (quaternion) start after position (x,y,z)
            self._mj_data.qpos[rot_adrr:rot_adrr + 4] = new_rotation_quat
        
        # self.simulate_steps(1)
        # time.sleep(3)
        # Step the simulation to apply the changes
        self.simulate_steps(10)
    def select_body(self, body_name: str):
        """
        Select a body in the MuJoCo viewer. Its label will appear if labels for
        selected objects are enabled in the viewer (usually by pressing 'L').
        To deselect, pass None as the body_name.
        """
        if self.render_mode != 'human' or not hasattr(self._env.renderer, "viewer"):
            print("Warning: Body selection is only available in 'human' render mode.")
            return

        viewer = self._env.renderer.viewer
        with viewer.lock():
            if body_name:
                body_id = mj.mj_name2id(self._mj_model, mj.mjtObj.mjOBJ_BODY, body_name)
                if body_id != -1:
                    # Set the selected body ID. The viewer will highlight it.
                    viewer._pert.select = body_id
                    # 'active' is for enabling mouse perturbations. Set to 0 to only select.
                    viewer._pert.active = 0
                else:
                    print(f"Warning: Body '{body_name}' not found. Deselecting.")
                    viewer._pert.select = -1  # Deselect if name not found
                    viewer._pert.active = 0
            else:
                # Deselect if body_name is None
                viewer._pert.select = -1
                viewer._pert.active = 0
    def select_free_joint(self, joint_name: str):
        """
        Select a free joint in the MuJoCo viewer. Its label will appear if labels for
        selected objects are enabled in the viewer.
        To deselect, pass None as the joint_name.
        """
        if self.render_mode != 'human' or not hasattr(self._env.renderer, "viewer"):
            print("Warning: Joint selection is only available in 'human' render mode.")
            return

        viewer = self._env.renderer.viewer
        with viewer.lock():
            if joint_name:
                joint_id = mj.mj_name2id(self._mj_model, mj.mjtObj.mjOBJ_JOINT, joint_name)
                if joint_id != -1:
                    # Set the selected joint ID and type.
                    viewer._pert.select = joint_id
                    viewer._pert.objtype = mj.mjtObj.mjOBJ_JOINT
                    viewer._pert.active = 0
                else:
                    print(f"Warning: Joint '{joint_name}' not found. Deselecting.")
                    viewer._pert.select = -1
                    viewer._pert.objtype = 0
                    viewer._pert.active = 0
            else:
                # Deselect if joint_name is None
                viewer._pert.select = -1
                viewer._pert.objtype = 0
                viewer._pert.active = 0
    # def select_free_joint(self, joint_name: str):
    #     """
    #     Select a free joint in the MuJoCo viewer. Its label will appear if labels for
    #     selected objects are enabled in the viewer (usually by pressing 'L').
    #     To deselect, pass None as the joint_name.
    #     """
    #     if self.render_mode != 'human' or not hasattr(self._env.renderer, "viewer"):
    #         print("Warning: Joint selection is only available in 'human' render mode.")
    #         return

    #     viewer = self._env.renderer.viewer
    #     with viewer.lock():
    #         if joint_name:
    #             joint_id = mj.mj_name2id(self._mj_model, mj.mjtObj.mjOBJ_JOINT, joint_name)
    #             if joint_id != -1:
    #                 # Set the selected joint ID. The viewer will highlight it.
    #                 viewer._pert.select = joint_id
    #                 # 'active' is for enabling mouse perturbations. Set to 0 to only select.
    #                 viewer._pert.active = 0
    #             else:
    #                 print(f"Warning: Joint '{joint_name}' not found. Deselecting.")
    #                 viewer._pert.select = -1  # Deselect if name not found
    #                 viewer._pert.active = 0
    #         else:
    #             # Deselect if joint_name is None
    #             viewer._pert.select = -1
    #             viewer._pert.active = 0
        
def convert_mj_struct_to_namedtuple(mj_struct):
    """
    convert a mujoco struct to a dictionary
    """
    attrs = [attr for attr in dir(mj_struct) if not attr.startswith('__') and not callable(getattr(mj_struct, attr))]
    return namedtuple(mj_struct.__class__.__name__, attrs)(**{attr: getattr(mj_struct, attr) for attr in attrs})





