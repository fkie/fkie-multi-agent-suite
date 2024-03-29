#!/usr/bin/env python
import os

if 'ROS_VERSION' in os.environ and os.environ['ROS_VERSION'] == '1':
  
    from setuptools import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup(
        # don't do this unless you want a globally visible script
        # scripts=['bin/myscript'],
        packages=['fkie_mas_discovery'],
        package_dir={'': 'src'}
    )
    setup(**d)
