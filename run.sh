roscore
rosrun lias_anp simulator.py
rosrun lias_anp traj_generator.py 
roscd lias_anp && rviz -d ./rviz/sim.rviz 