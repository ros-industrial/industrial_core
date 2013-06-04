#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   ##  don't do this unless you want a globally visible script
   scripts=['industrial_robot_simulator'],
   packages=['industrial_robot_simulator'],
   package_dir={'': ''}
)

setup(**d)
