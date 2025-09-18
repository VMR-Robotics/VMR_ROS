# 蓝芯SDK ROS2程序

本模块通过sdk可以控制底盘  程序目前仅支持从源代码构建

### 配置
  配置文件： vmr_ros_pkg/config/lx_config.yaml

  | 配置项          | 类型     | 举例          | 说明                                                         |
  | --------------- | -------- | ------------- | ------------------------------------------------------------ |
  | ip              | string   | "127.0.0.1"     | 蓝芯机器人IP                                             |
  | host            | int      | 9000            | 蓝芯机器人端口(默认为9000)                                |
  | twist           | string    | "twist"        | 订阅速度控制 topic  name                                      |



### 启动步骤

1. 将文件放在ros2工作区下

2. 输入指令：source /opt/ros/$ROS_VERSION/setup.bash

3. 输入指令：colcon build --packages-select vmr_ros_pkg

4. 输入指令：source install/setup.bash

5. 输入指令：ros2 launch vmr_ros_pkg lx_bringup.launch.py



### 接口
#### ros2 订阅
| 接口               | 类型               | 数据类型               | 说明                                                         |
| ------------------ | ------------------ | --------------------- | ------------------------------------------------------------ |
| $twist             | Subscriber topic   | geometry_msgs/msg/Twist    | 订阅速度控制                                         |
| $twist             | Service            | geometry_msgs/msg/Twist    | 订阅速度控制                                         |



#### ros2 发布
| 接口               | 类型         | 数据类型                | 说明                                                         |
| ------------------ | ------------ | ----------------------- | ------------------------------------------------------------ |
| /vmr/robot_odometry | ros2 topic   | nav_msgs/msg/Odometry      | 发布里程计数据                                             |
| /vmr/robot_imu  | ros2 topic   | sensor_msgs/msg/Imu  | 发布IMU状态                                             |
| /vmr/robot_battery  | ros2 topic   | sensor_msgs/msg/BatteryState | 发布电池状态                                             |
| /vmr/robot_light | ros2 topic   | vmr/msg/Light  | 发布灯带状态                                             |
| /vmr/robot_lift | ros2 topic   | vmr_ros_pkg/msg/LiftState | 发布顶升状态                                              |
| /vmr/robot_exception | ros2 topic   | vmr/msg/RobotException | 发布异常状态                                              |


