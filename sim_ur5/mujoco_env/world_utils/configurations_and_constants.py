import numpy as np

from sim_ur5.mujoco_env.tasks.null_task import NullTask
from sim_ur5.mujoco_env.episode import *

scene = SceneSpec(
    'clairlab',
    objects=(
        ObjectSpec('bin_dark_wood', base_pos=[0.2, -0.3, 0]),
        ObjectSpec('milk', base_pos=[0.2, -0.3, 0.1], base_joints=(JointSpec('free'),)),
    ),
    render_camera='top-right',
    init_keyframe='home'
)

# This is the configuration for the mujoco environment for the UR5e robot with two UR5e robots and a battery object.
# cfg
muj_env_config = dict(
    scene=scene,
    # scene=dict(
    #     resource='clairlab',
    #     render_camera='top-right'
    # ),
    robots=dict(
        ur5e_1=dict(
            resource='ur5e',
            attachments=['adhesive_gripper'],
            base_pos=[0, 0, 0.01],
            base_rot=[0, 0, 1.57079632679],
            privileged_info=True,
        ),
        ur5e_2=dict(
            resource='ur5e',
            attachments=['adhesive_gripper'],
            base_pos=[-0.76, -1.33, 0.01],
            base_rot=[0, 0, -1.57079632679],
            privileged_info=True,
        ),
    ),
    tasks=dict(
        ur5e_1=NullTask,
        ur5e_2=NullTask,
    ),
)

INIT_MAX_VELOCITY = np.array([3]*6)

# relative position of grasped object from end effector
grasp_offset = 0.02

frame_skip = 5
