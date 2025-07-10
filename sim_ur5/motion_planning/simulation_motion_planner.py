import os

from klampt import Geometry3D
from klampt.model.geometry import box
from klampt import vis

from motion_planner.motion_planner import AbstractMotionPlanner
from .configurations import *


class SimulationMotionPlanner(AbstractMotionPlanner):
    def __init__(self):
        """
        Initialize the simulation motion planner.
        """
        super().__init__(ee_offset=0.00, eps=0.05)
        self.ee_link = self.ur5e_2.link("ee_link")  # End-effector link

    def _get_klampt_world_path(self):
        """
        Get the path to the Klampt world XML file.
        """
        dir = os.path.dirname(os.path.realpath(__file__))
        world_path = os.path.join(dir, "klampt_world.xml")
        return world_path

    def _add_attachments(self, robot, attachments):
        """
        Add attachments to the robot (not implemented).
        """
        pass

    def attach_box_to_ee(self):
        """
        Attach a box to the end effector for collision detection.
        Should be called once to set up the collision geometry.
        """
        # Note that the order is different here, width is in the z direction
        sx, sy, sz = block_size
        box_obj = box(width=sz, height=sy, depth=sx, center=[0, 0, grasp_offset])
        box_geom = Geometry3D()
        box_geom.set(box_obj)

        # Set the geometry of the end effector link
        self.ee_link.geometry().set(box_geom)

    def detach_box_from_ee(self):
        """
        Detach the box from the end effector by replacing it with a dummy box.
        """
        dummy_box_obj = box(width=0.001, height=0.001, depth=0.001, center=[0, 0, 0])
        dummy_box_geom = Geometry3D()
        dummy_box_geom.set(dummy_box_obj)

        # Replace the end effector geometry with the dummy box
        self.ee_link.geometry().set(dummy_box_geom)

    def move_block(self, name, position):
        """
        Move a block to a specified position in the world.
        :param name: Name of the block.
        :param position: Target position for the block.
        """
        rigid_obj = self.world.rigidObject(name)  # Get the rigid object by name
        width, depth, height = block_size
        box_obj = box(width=width, height=height, depth=depth, center=position)
        rigid_obj.geometry().set(box_obj)  # Update the geometry of the block

    def add_block(self, name, position, color=(0.3, 0.3, 0.3, 0.8)):
        """
        Add a block to the world at a specified position with a given color.
        :param name: Name of the block.
        :param position: Position of the block.
        :param color: Color of the block (RGBA format).
        """
        self._add_box_geom(name, block_size, position, color)

    def _add_box_geom(self, name, size, center, color, update_vis=True):
        """
        Add box geometry for collision in the world.
        :param name: Name of the box.
        :param size: Size of the box (width, depth, height).
        :param center: Center position of the box.
        :param color: Color of the box (RGBA format).
        :param update_vis: Whether to update the visualization.
        """
        width, depth, height = size
        box_obj = box(width=width, height=height, depth=depth, center=center)
        box_geom = Geometry3D()
        box_geom.set(box_obj)

        # Create a rigid object in the world and set its geometry and appearance
        box_rigid_obj = self.world.makeRigidObject(name)
        box_rigid_obj.geometry().set(box_geom)
        box_rigid_obj.appearance().setColor(*color)

        # Add the box to the visualization if required
        if update_vis:
            vis.add("world", self.world)
