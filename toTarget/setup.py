#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['toTarget'],  # Replace with your actual package name
    package_dir={'': 'src'}  # Replace 'src' with the directory containing your Python scripts
)

setup(**setup_args)

