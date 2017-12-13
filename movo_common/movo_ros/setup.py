#!/usr/bin/env python
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['movo','movo_action_clients','movo_gripper_interface','movo_joint_interface','movo_jtas'],
    package_dir={'': 'src'})

setup(**setup_args)
