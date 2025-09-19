#!/bin/bash
pkill -f -9 vmr_ros_pkg
source ./install/setup.bash

ros2 launch vmr_ros_pkg vmr_bringup.launch.py