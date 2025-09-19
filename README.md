<p align="right">
  <strong>English</strong> | <a href="./README.zh-CN.md">简体中文</a>
</p>
# BlueCore SDK ROS2 Package

This module controls the robot chassis through the SDK; **currently the package must be built from source**.

---

## 1. Configuration

**File:** `vmr_ros_pkg/config/vmr_config.yaml`

| Key   | Type   | Example     | Description |
|-------|--------|-------------|-------------|
| `ip`    | string | `"127.0.0.1"` | IP address of the BlueCore robot |
| `host`  | int    | `9000`        | BlueCore robot port (default 9000) |
| `twist` | string | `"twist"`     | Topic name for velocity commands |

---

## 2. Build & Launch

1. Clone / copy the package into your ROS 2 workspace.
2. Build and source:

```bash
source /opt/ros/$ROS_VERSION/setup.bash
colcon build --packages-select vmr_ros_pkg
source install/setup.bash
ros2 launch vmr_ros_pkg vmr_bringup.launch.py
```

## 3. ROS 2 Interfaces

### 3.1 Subscribed Topics
| Topic (replace `$twist` with value in yaml) | Type | Message | 
| ------------------------------------------- | ------------ | ------------------------- | 
| `$twist` | subscription | `geometry_msgs/msg/Twist` |
### 3.2 Services
| Name | Type | Request / Response | 
| ------------------------ | ------- | ---------------------------- | 
| `/vmr/robot_lift_ctrl` | service | `vmr_ros_pkg/srv/LiftCtrl`           | 
| `/vmr/robot_light_ctrl` | service | `vmr_ros_pkg/srv/LightCtrl`         | 
| `/vmr/robot_task_result` | service | `vmr_ros_pkg/srv/TaskResult`       |

### 3.3 Published Topics

| Topic | Message Type | Description | 
| ---------------------- | ------------------------------ | ---------------- | 
| `/vmr/robot_odometry` | `nav_msgs/msg/Odometry` | Odometry | 
| `/vmr/robot_imu` | `sensor_msgs/msg/Imu` | IMU data | 
| `/vmr/robot_battery` | `sensor_msgs/msg/BatteryState` | Battery level | 
| `/vmr/robot_light` | `vmr/msg/Light` | LED strip status | 
| `/vmr/robot_lift` | `vmr_ros_pkg/msg/LiftState` | Lift state | 
| `/vmr/robot_exception` | `vmr/msg/RobotException` | Exception info |

## 4. Communication Protocol
Uses [rest_rpc](https://github.com/qicosmos/rest_rpc) for TCP-based communication with the VMR robot.