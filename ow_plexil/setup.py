# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
#The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
#Research and Simulation can be found in README.md in the root directory of
#this repository.

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
  packages=['rqt_plexil_plan_selection'],
  package_dir={'': 'rqt_plexil_plan_selection/src'},
  requires=['std_msgs', 'rospy']
)

setup(**setup_args)
