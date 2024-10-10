# lias_anp

roscore
rosrun lias_anp simulator.py
rosrun lias_anp traj_generator_bigeight.py 
rosrun lias_anp traj_generator_square.py 

roscd lias_anp && rviz -d ./rviz/sim.rviz 

# if you want to use manual mode 
roslaunch lias_anp teleop_joy.launch
roslaunch lias_anp teleop_keyboard.launch

# import sys, roslib, os
# project_root = roslib.packages.get_pkg_dir('lias_anp')
# root_dir = os.path.abspath(os.path.join(project_root, 'scripts'))
# sys.path.append(root_dir)

# Open sonar recorder
rosrun lias_anp sonar_data_processor.py 
