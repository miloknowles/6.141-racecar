## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
# https://github.com/mehditlili/boost-python-catkin-example/blob/master/setup.py

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['lab6'],
    package_dir={'': 'src'},
)

setup(**setup_args)
