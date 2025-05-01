from setuptools import setup, find_packages
import glob
import os

# Get MuJoCo env files
mujoco_env_files = glob.glob('sim_ur5/mujoco_env/assets/**/*', recursive=True)
mujoco_env_files = [f.replace('sim_ur5/mujoco_env/', '') for f in mujoco_env_files]

# Get motion planning files
motion_planning_files = glob.glob('sim_ur5/motion_planning/assets/ur5_rob/**/*', recursive=True)
motion_planning_files = [f.replace('sim_ur5/motion_planning/', '') for f in motion_planning_files]
motion_planning_files.extend(['klampt_world.xml', "ur5.urdf"])

setup(
    name='AIR-24',
    version='0.1.0',
    packages=find_packages(),
    python_requires='>=3.10',
    install_requires=[
        'numpy>=1.26.4',
        'opencv-python>=4.10.0.82',
        'pycurl>=7.45.3',
        'PyOpenGL>=3.1.7',
        'PyQt5>=5.15.10',
        'PyQt5-Qt5>=5.15.2',
        'PyQt5-sip>=12.13.0',
        'pyrealsense2>=2.55.1.6486',
        'ur-rtde>=1.5.8',
        'chime>=0.7.0',
        'mujoco>=3.2.2',
        'gymnasium>=0.29.1',
        'dm_control>=1.0.22',
        'pyYAML>=6.0.1',
        'imageio>=2.34.2',
        'frozendict>=2.4.4',
        'Klampt>=0.9.2'
    ],
    package_data={
        'sim_ur5.mujoco_env': mujoco_env_files,
        'sim_ur5.motion_planning': motion_planning_files,
    }
)
