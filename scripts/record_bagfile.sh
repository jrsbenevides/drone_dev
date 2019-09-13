#!/bin/bash


echo "Recording topics"

rosbag record -a -x "/bebop/image_raw|/bebop/image_raw/theora|/bebop/image_raw/theora/(.*)" $*

