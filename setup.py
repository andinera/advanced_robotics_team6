#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['advanced_robotics_team6']
d['package_dir'] = {'': '..'}

setup(**d)