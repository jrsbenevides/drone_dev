#!/bin/bash

clear

echo "Changing source to bebop_ws"
#source ~/Dropbox/Projects/bebop_ws/devel/setup.bash
file="$(find -name bebop_ws)/devel/setup.bash"
source $file

echo "Launching bebop_node.launch"
roslaunch bebop_tools joy_teleop.launch

echo "Changing source to catkin_ws"
source ~/catkin_ws/devel/setup.bash



