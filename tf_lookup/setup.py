#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tf_lookup'],
    package_dir={'': 'src'},
    requires=['actionlib', 'tf']
)

setup(**d)
