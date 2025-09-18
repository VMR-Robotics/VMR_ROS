#!/bin/bash
ros2 service call /vmr/robot_lift_ctrl vmr_ros_pkg/srv/LiftCtrl  '{height: 50}'
sleep 1
ros2 service call /vmr/robot_light_ctrl vmr_ros_pkg/srv/LightCtrl  '{light_state: 1}'
sleep 1
ros2 service call /vmr/robot_light_ctrl vmr_ros_pkg/srv/LightCtrl  '{light_state: 2}'