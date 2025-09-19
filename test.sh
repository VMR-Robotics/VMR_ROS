#!/bin/bash
echo "start test lift"
echo "test lift up"
ros2 service call /vmr/robot_lift_ctrl vmr_ros_pkg/srv/LiftCtrl  '{height: 100}'
sleep 5
echo "test lift down"
ros2 service call /vmr/robot_lift_ctrl vmr_ros_pkg/srv/LiftCtrl  '{height: 0}'
sleep 5
echo "start test light"

echo "send light to red "
ros2 service call /vmr/robot_light_ctrl vmr_ros_pkg/srv/LightCtrl  '{light_state: 7}'
sleep 5

echo "send light to green "
ros2 service call /vmr/robot_light_ctrl vmr_ros_pkg/srv/LightCtrl  '{light_state: 8}'
sleep 5

echo "send light to blue "
ros2 service call /vmr/robot_light_ctrl vmr_ros_pkg/srv/LightCtrl  '{light_state: 11}'