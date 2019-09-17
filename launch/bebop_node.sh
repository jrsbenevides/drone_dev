#!/bin/bash

clear

echo "Changing source to bebop_ws"
#source ~/Dropbox/Projects/bebop_ws/devel/setup.bash
file="$(find -name bebop_ws)/devel/setup.bash"
source $file

echo "Launching bebop_node.launch"
roslaunch bebop_driver bebop_node.launch

rosparam set /bebop/bebop_driver/PictureSettingsVideoStabilizationModeMode 3

echo "Changing source to catkin_ws"
source ~/Dropbox/Projects/catkin_ws/devel/setup.bash



