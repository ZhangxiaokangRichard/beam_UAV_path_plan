#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["uav_guide"],
    package_dir={"": "."},
)

setup(**setup_args)
