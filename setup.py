#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['sorting_robot'],
    package_dir={'': 'src'},
    install_requires=['numpy', 'matplotlib', 'pillow']
)

setup(**setup_args)
