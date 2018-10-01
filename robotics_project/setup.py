from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=[	'scripts/state_machines/sm_students.py', 
  		   	'scripts/utils/poses_pub.py', 
  		   	'scripts/utils/gazebo_models_hdl.py', 
  		   	'scripts/demo/manipulation_server.py',
        	'scripts/demo/manipulation_client.py',
        	'scripts/demo/grasps_server.py'])
setup(**d)