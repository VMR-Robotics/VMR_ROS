#ifndef VMR_DATA_H
#define VMR_DATA_H
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "vmr_ros_pkg/msg/servo_state.hpp"
#include "vmr_ros_pkg/srv/lift_ctrl.hpp"
#include "vmr_ros_pkg/srv/light_ctrl.hpp"
#include "vmr_ros_pkg/msg/lift_state.hpp"
#include "vmr_ros_pkg/msg/exception.hpp"
#include "vmr_ros_pkg/msg/robot_exception.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <msgpack.hpp>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "vmr_ros_pkg/msg/light.hpp"
#include "vmr_ros_pkg/msg/robot_state.hpp"
#include "vmr_ros_pkg/msg/dog_state.hpp"

inline float normalize(float angle) {
  if (std::fabs(angle) > M_PI * 2) { 
    angle = angle - long(angle / (M_PI * 2)) * M_PI * 2;
  }
  while (angle > M_PI) {
    angle -= M_PI * 2;
  }
  while (angle <= -M_PI) {
    angle += M_PI * 2;
  }
  return angle;
}


// VMR Header 
// Implemented overloads to convert vmr Header to/from std_msgs::msg::Header.
struct Header {
  uint32_t seq{0};
  uint64_t timestamp_ns;
  std::string frame_id;
  MSGPACK_DEFINE(seq, timestamp_ns, frame_id)
  Header &operator=(const std_msgs::msg::Header &ros_header) {
    frame_id = ros_header.frame_id;
    timestamp_ns = static_cast<uint64_t>(ros_header.stamp.sec) * 1000000000ULL +
                   ros_header.stamp.nanosec;
    return *this;
  }
  operator std_msgs::msg::Header() const {
    std_msgs::msg::Header data;
    data.stamp.sec = static_cast<uint32_t>(timestamp_ns / 1000000000ULL);
    data.stamp.nanosec = static_cast<uint32_t>(timestamp_ns % 1000000000ULL);
    data.frame_id = frame_id;
    return data;
  }
};
// VMR Vector3 
// Implemented overloads to convert vmr Vector3 to/from geometry_msgs::msg::Vector3.
struct Vector3 {
  double x, y, z;
  MSGPACK_DEFINE(x, y, z)
  Vector3 &operator=(const geometry_msgs::msg::Vector3 &other) {
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
  }
  operator geometry_msgs::msg::Vector3() const {
    geometry_msgs::msg::Vector3 data;
    data.x = x;
    data.y = y;
    data.z = z;
    return data;
  }
};

// VMR Point 
// Implemented overloads to convert vmr Point to/from geometry_msgs::msg::Point.
struct Point {
  double x, y, z;
  MSGPACK_DEFINE(x, y, z)
  Point &operator=(const geometry_msgs::msg::Point &other) {
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
  }
  operator geometry_msgs::msg::Point() const {
    geometry_msgs::msg::Point data;
    data.x = x;
    data.y = y;
    data.z = z;
    return data;
  }
};

// VMR Quaternion 
// Implemented overloads to convert vmr Quaternion to/from geometry_msgs::msg::Quaternion.
struct Quaternion {
  double x, y, z, w;
  MSGPACK_DEFINE(x, y, z, w)
  Quaternion &operator=(const geometry_msgs::msg::Quaternion &other) {
    x = other.x;
    y = other.y;
    z = other.z;
    w = other.w;
    return *this;
  }
  operator geometry_msgs::msg::Quaternion() const {
    geometry_msgs::msg::Quaternion data;
    data.x = x;
    data.y = y;
    data.z = z;
    data.w = w;
    return data;
  }
};


// VMR Pose 
// Implemented overloads to convert VMR Pose to/from geometry_msgs::msg::Pose.
struct Pose {
  Point position;
  Quaternion orientation;
  MSGPACK_DEFINE(position, orientation)
  Pose &operator=(const geometry_msgs::msg::Pose &other) {
    position = other.position;
    orientation = other.orientation;
    return *this;
  }
  operator geometry_msgs::msg::Pose() const {
    geometry_msgs::msg::Pose data;
    data.position = position;
    data.orientation = orientation;
    return data;
  }
};


struct Twist {
  Vector3 linear;
  Vector3 angular;
  Twist() {}
  Twist(const geometry_msgs::msg::Twist &data) {
    linear = data.linear;   
    angular = data.angular; 
  }
  Twist &operator=(const geometry_msgs::msg::Twist &other) {
    linear = other.linear; 
    angular = other.angular; 
    return *this;
  }
  operator geometry_msgs::msg::Twist() const {
    geometry_msgs::msg::Twist ros_twist;
    ros_twist.linear = linear;
    ros_twist.angular = angular;
    return ros_twist;
  }
  MSGPACK_DEFINE(linear, angular)
};

// 里程计数据 (纯基本类型)
struct BasicOdometry {
  Header header;
  std::string child_frame_id;
  Pose pose;

  Twist twist;

  MSGPACK_DEFINE(header, child_frame_id, pose, twist)


  static BasicOdometry fromRos(const nav_msgs::msg::Odometry &ros_msg) {
    BasicOdometry odom;
    odom.header.frame_id = ros_msg.header.frame_id;
    odom.header.timestamp_ns =
        static_cast<uint64_t>(ros_msg.header.stamp.sec) * 1'000'000'000ULL +
        ros_msg.header.stamp.nanosec;
    odom.child_frame_id = ros_msg.child_frame_id;

    odom.pose.position.x = ros_msg.pose.pose.position.x;
    odom.pose.position.y = ros_msg.pose.pose.position.y;
    odom.pose.position.z = ros_msg.pose.pose.position.z;
    odom.pose.orientation = {
        ros_msg.pose.pose.orientation.x, ros_msg.pose.pose.orientation.y,
        ros_msg.pose.pose.orientation.z, ros_msg.pose.pose.orientation.w};

    odom.twist.linear.x = ros_msg.twist.twist.linear.x;
    odom.twist.linear.y = ros_msg.twist.twist.linear.y;
    odom.twist.linear.z = ros_msg.twist.twist.linear.z;
    odom.twist.angular.x = ros_msg.twist.twist.angular.x;
    odom.twist.angular.y = ros_msg.twist.twist.angular.y;
    odom.twist.angular.z = ros_msg.twist.twist.angular.z;
    return odom;
  }

  nav_msgs::msg::Odometry toRos() const {
    nav_msgs::msg::Odometry ros_msg;
    ros_msg.header.frame_id = header.frame_id;
    ros_msg.child_frame_id = child_frame_id;
    uint64_t secs = header.timestamp_ns / 1000000000ULL;
    uint32_t nsecs = header.timestamp_ns % 1000000000ULL;

    ros_msg.header.stamp.sec = static_cast<int32_t>(secs);
    ros_msg.header.stamp.nanosec = static_cast<uint32_t>(nsecs);
    ros_msg.pose.pose.position.x = pose.position.x;
    ros_msg.pose.pose.position.y = pose.position.y;
    ros_msg.pose.pose.position.z = pose.position.z;
    ros_msg.pose.pose.orientation.x = pose.orientation.x;
    ros_msg.pose.pose.orientation.y = pose.orientation.y;
    ros_msg.pose.pose.orientation.z = pose.orientation.z;
    ros_msg.pose.pose.orientation.w = pose.orientation.w;

    ros_msg.twist.twist.linear.x = twist.linear.x;
    ros_msg.twist.twist.linear.y = twist.linear.y;
    ros_msg.twist.twist.linear.z = twist.linear.z;
    ros_msg.twist.twist.angular.x = twist.angular.x;
    ros_msg.twist.twist.angular.y = twist.angular.y;
    ros_msg.twist.twist.angular.z = twist.angular.z;

    return ros_msg;
  }

  std::string toString() const {
    std::ostringstream oss;

    oss << std::fixed << std::setprecision(3);
    oss << "Odometry Data:\n";
    oss << "  Header:\n";
    oss << "    Frame ID: " << header.frame_id << "\n";
    oss << "    Timestamp: " << header.timestamp_ns / 1000000000ULL << "."
        << header.timestamp_ns % 1000000000ULL << "\n";
    oss << "  Pose:\n";
    oss << "    Position: [" << pose.position.x << ", " << pose.position.y
        << ", " << pose.position.z << "]\n";
    oss << "    Orientation: [" << pose.orientation.x << ", "
        << pose.orientation.y << ", " << pose.orientation.z << ", "
        << pose.orientation.w << "]\n";

    oss << "  Twist:\n";
    oss << "    Linear: [" << twist.linear.x << ", " << twist.linear.y << ", "
        << twist.linear.z << "]\n";
    oss << "    Angular: [" << twist.angular.x << ", " << twist.angular.y
        << ", " << twist.angular.z << "]\n";
    oss << "  Child Frame ID: " << child_frame_id << "\n";

    return oss.str();
  }
};

struct BasicLaserScan {

  Header header;
  // 扫描参数
  float angle_min;     
  float angle_max;      
  float angle_increment; 
  float time_increment;  
  float scan_time;      
  float range_min;      
  float range_max;      

  std::vector<float> ranges;     
  std::vector<float> intensities; 

  MSGPACK_DEFINE(header, angle_min, angle_max, angle_increment, time_increment,
                 scan_time, range_min, range_max, ranges, intensities)

  static BasicLaserScan fromRos(const sensor_msgs::msg::LaserScan &ros_msg) {
    BasicLaserScan scan;
    scan.header.frame_id = ros_msg.header.frame_id;
    scan.header.timestamp_ns =
        static_cast<uint64_t>(ros_msg.header.stamp.sec) * 1'000'000'000ULL +
        ros_msg.header.stamp.nanosec;
    scan.angle_min = ros_msg.angle_min;
    scan.angle_max = ros_msg.angle_max;
    scan.angle_increment = ros_msg.angle_increment;
    scan.time_increment = ros_msg.time_increment;
    scan.scan_time = ros_msg.scan_time;
    scan.range_min = ros_msg.range_min;
    scan.range_max = ros_msg.range_max;
    scan.ranges = ros_msg.ranges;
    scan.intensities = ros_msg.intensities;
    return scan;
  }

  sensor_msgs::msg::LaserScan toRos() const {
    sensor_msgs::msg::LaserScan ros_msg;
    ros_msg.header.frame_id = header.frame_id;
    uint64_t secs = header.timestamp_ns / 1000000000ULL;
    uint32_t nsecs = header.timestamp_ns % 1000000000ULL;
    ros_msg.header.stamp.sec = static_cast<int32_t>(secs);
    ros_msg.header.stamp.nanosec = static_cast<uint32_t>(nsecs);
    ros_msg.angle_min = angle_min;
    ros_msg.angle_max = angle_max;
    ros_msg.angle_increment = angle_increment;
    ros_msg.time_increment = time_increment;
    ros_msg.scan_time = scan_time;
    ros_msg.range_min = range_min;
    ros_msg.range_max = range_max;
    ros_msg.ranges = ranges;
    ros_msg.intensities = intensities;
    return ros_msg;
  }

  size_t size() const { return ranges.size(); }
  bool hasIntensities() const { return !intensities.empty(); }

  size_t validCount() const {
    size_t count = 0;
    for (float r : ranges) {
      if (!std::isnan(r) && !std::isinf(r) && r >= range_min &&
          r <= range_max) {
        count++;
      }
    }
    return count;
  }
};

struct Orientation {
  double x;
  double y;
  double z;
  double w;
  MSGPACK_DEFINE(x, y, z, w)
};

struct BasicPose {
  Header header;
  Pose pose;
  MSGPACK_DEFINE(header, pose)
  static BasicPose
  fromRos(const geometry_msgs::msg::PoseStamped &pose_stamped) {
    BasicPose basic_pose;
    basic_pose.header.frame_id = pose_stamped.header.frame_id;
    basic_pose.header.timestamp_ns =
        static_cast<uint64_t>(pose_stamped.header.stamp.sec) *
            1'000'000'000ULL +
        pose_stamped.header.stamp.nanosec;
    basic_pose.pose.position.x = pose_stamped.pose.position.x;
    basic_pose.pose.position.y = pose_stamped.pose.position.y;
    basic_pose.pose.position.z = pose_stamped.pose.position.z;
    basic_pose.pose.orientation.x = pose_stamped.pose.orientation.x;
    basic_pose.pose.orientation.y = pose_stamped.pose.orientation.y;
    basic_pose.pose.orientation.z = pose_stamped.pose.orientation.z;
    basic_pose.pose.orientation.w = pose_stamped.pose.orientation.w;
    return basic_pose;
  }
};

struct LocState {
  BasicOdometry basic_odom;
  int exception{0};
  float weight{0};
  MSGPACK_DEFINE(basic_odom, exception, weight)

  static vmr_ros_pkg::msg::RobotState toRos(const LocState &robot_relocate) {
    vmr_ros_pkg::msg::RobotState data;
    data.header.frame_id = robot_relocate.basic_odom.header.frame_id;
    uint64_t secs =
        robot_relocate.basic_odom.header.timestamp_ns / 1000000000ULL;
    uint32_t nsecs =
        robot_relocate.basic_odom.header.timestamp_ns % 1000000000ULL;

    data.header.stamp.sec = static_cast<int32_t>(secs);
    data.header.stamp.nanosec = static_cast<uint32_t>(nsecs);
    data.pose.x = robot_relocate.basic_odom.pose.position.x;
    data.pose.y = robot_relocate.basic_odom.pose.position.y;
    float q_norm =
        std::sqrt(std::pow(robot_relocate.basic_odom.pose.orientation.z, 2) +
                  std::pow(robot_relocate.basic_odom.pose.orientation.w, 2));
    float new_z = robot_relocate.basic_odom.pose.orientation.z / q_norm;
    float new_w = robot_relocate.basic_odom.pose.orientation.w / q_norm;
    data.pose.theta = normalize(std::atan2(new_z, new_w) * 2.f);
    data.exception = robot_relocate.exception;
    data.weight = robot_relocate.weight;
    return data;
  }
};

struct PoseWithCovariance {
  Pose pose;
  std::vector<double> covariance;
  MSGPACK_DEFINE(pose, covariance)
};

struct PoseWithCovarianceStamped {
  Header header;
  PoseWithCovariance pose;
  MSGPACK_DEFINE(header, pose)
  static PoseWithCovarianceStamped
  fromRos(const geometry_msgs::msg::PoseWithCovarianceStamped &pose_stamped) {
    PoseWithCovarianceStamped data;
    data.header.frame_id = pose_stamped.header.frame_id;
    data.header.timestamp_ns =
        static_cast<uint64_t>(pose_stamped.header.stamp.sec) *
            1'000'000'000ULL +
        pose_stamped.header.stamp.nanosec;
    data.pose.pose.position.x = pose_stamped.pose.pose.position.x;
    data.pose.pose.position.y = pose_stamped.pose.pose.position.y;
    data.pose.pose.position.z = pose_stamped.pose.pose.position.z;
    data.pose.pose.orientation.x = pose_stamped.pose.pose.orientation.x;
    data.pose.pose.orientation.y = pose_stamped.pose.pose.orientation.y;
    data.pose.pose.orientation.z = pose_stamped.pose.pose.orientation.z;
    data.pose.pose.orientation.w = pose_stamped.pose.pose.orientation.w;
    data.pose.covariance.resize(36);
    for (size_t i = 0; i < 36; ++i) {
      data.pose.covariance[i] = pose_stamped.pose.covariance[i];
    }
    return data;
  }
};

struct TwistStamped {
  Header header;
  Twist twist;
  MSGPACK_DEFINE(header, twist)
  TwistStamped() {}
  TwistStamped(const geometry_msgs::msg::TwistStamped &data) {
    header = data.header;
    twist = data.twist;
  }
  TwistStamped &operator=(const geometry_msgs::msg::TwistStamped &data) {
    header = data.header;
    twist = data.twist;
    return *this;
  }

  operator geometry_msgs::msg::TwistStamped() const {
    geometry_msgs::msg::TwistStamped data;
    data.header = header;
    data.twist = twist;
    return data;
  }
};

struct BatteryState {
  Header header;
  float voltage;              // 电压
  float current;              // 电流
  float charge;               // 充电
  float capacity;             // 电池容量
  float design_capacity;      // 额定电量
  float percentage;           // 电量百分比
  int8_t power_supply_status; // 电池状态
  std::string serial_number;  // 序列号

  MSGPACK_DEFINE(header, voltage, current, charge, capacity, design_capacity,
                 percentage, power_supply_status, serial_number)

  BatteryState operator=(const sensor_msgs::msg::BatteryState &data) {
    header = data.header;
    voltage = data.voltage;
    current = data.current;
    charge = data.charge;
    capacity = data.capacity;
    design_capacity = data.design_capacity;
    percentage = data.percentage;
    power_supply_status = data.power_supply_status;
    serial_number = data.serial_number;

    return *this;
  }

  operator sensor_msgs::msg::BatteryState() const {
    sensor_msgs::msg::BatteryState data;
    data.header = header;
    data.voltage = voltage;
    data.current = current;
    data.charge = charge;
    data.capacity = capacity;
    data.design_capacity = design_capacity;
    data.percentage = percentage;
    data.power_supply_status = power_supply_status;
    data.serial_number = serial_number;
    return data;
  }
};

struct Imu {
  Header header;
  Quaternion orientation;
  Vector3 angular_velocity;
  Vector3 linear_acceleration;
  MSGPACK_DEFINE(header, orientation, angular_velocity, linear_acceleration)

  Imu &operator=(const sensor_msgs::msg::Imu &data) {
    header = data.header;
    orientation = data.orientation;
    angular_velocity = data.angular_velocity;
    linear_acceleration = data.linear_acceleration;
    return *this;
  }

  operator sensor_msgs::msg::Imu() const {
    sensor_msgs::msg::Imu data;
    data.header = header;
    data.orientation = orientation;
    data.angular_velocity = angular_velocity;
    data.linear_acceleration = linear_acceleration;
    return data;
  }
};

struct Light {
  Header header;
  int32_t light_state;
  MSGPACK_DEFINE(header, light_state)

  Light() {}
  Light(const vmr_ros_pkg::msg::Light &data) {
    header = data.header;
    light_state = data.light_state;
  }
  Light &operator=(const vmr_ros_pkg::msg::Light &data) {
    header = data.header;
    light_state = data.light_state;
    return *this;
  }

  operator vmr_ros_pkg::msg::Light() const {
    vmr_ros_pkg::msg::Light data;
    data.header = header;
    data.light_state = light_state;
    return data;
  }
};

struct ServoState {
  Header header;
  uint16_t error_code = 0;

  uint16_t voltage = 0;

  int16_t temperature = 0;

  float position = 0.0;
  float speed = 0.0;
  int16_t current = 0;
  bool moving_flag = false;
  bool enable_flag = false;
  uint8_t work_mode = 0;
  MSGPACK_DEFINE(header, error_code, voltage, temperature, position, speed,
                 current, moving_flag, enable_flag, work_mode)

  ServoState() {}
  ServoState(const vmr_ros_pkg::msg::ServoState &data) {
    header = data.header;
    error_code = data.error_code;
    voltage = data.voltage;
    temperature = data.temperature;
    position = data.position;
    speed = data.speed;
    current = data.current;
    moving_flag = data.moving_flag;
    enable_flag = data.enable_flag;
    work_mode = data.work_mode;
  }
  ServoState &operator=(const vmr_ros_pkg::msg::ServoState &data) {
    header = data.header;
    error_code = data.error_code;
    voltage = data.voltage;
    temperature = data.temperature;
    position = data.position;
    speed = data.speed;
    current = data.current;
    moving_flag = data.moving_flag;
    enable_flag = data.enable_flag;
    work_mode = data.work_mode;
    return *this;
  }

  operator vmr_ros_pkg::msg::ServoState() const {
    vmr_ros_pkg::msg::ServoState data;
    data.header = header;
    data.error_code = error_code;
    data.voltage = voltage;
    data.temperature = temperature;
    data.position = position;
    data.speed = speed;
    data.current = current;
    data.moving_flag = moving_flag;
    data.enable_flag = enable_flag;
    data.work_mode = work_mode;
    return data;
  }
};

struct JointState{
  Header header;
  std::vector<std::string> name;
  std::vector<double> position;
  std::vector<double> velocity;
  MSGPACK_DEFINE(header, name, position, velocity)
  JointState() {}
  JointState(const sensor_msgs::msg::JointState &data) {
    header = data.header;
    name = data.name;
    position = data.position;
    velocity = data.velocity; 
  }
  
  JointState &operator=(const sensor_msgs::msg::JointState &data) {
    header = data.header;
    name = data.name;
    position = data.position;
    velocity = data.velocity;
    return *this;
  }

  operator sensor_msgs::msg::JointState() const {
    sensor_msgs::msg::JointState data;
    data.header = header;
    return data;
  }
};
struct LiftState{
  Header header;
  int32_t height;
  MSGPACK_DEFINE(header, height)
  LiftState() {}
  LiftState(const vmr_ros_pkg::msg::LiftState &data) {
    header = data.header;
    height = data.height;
  }
  
  LiftState &operator=(const vmr_ros_pkg::msg::LiftState &data) {
    header = data.header;
    height = data.height;
    return *this;
  }

  operator vmr_ros_pkg::msg::LiftState() const {
    vmr_ros_pkg::msg::LiftState data;
    data.header = header;
    data.height = height;
    return data;
  }
};

struct DogState{
  Header header;
  int32_t dog_state;
  MSGPACK_DEFINE(header, dog_state)
  DogState() {}
  DogState(const vmr_ros_pkg::msg::DogState &data) {
    header = data.header;
    dog_state = data.dog_state;
  }
  
  DogState &operator=(const vmr_ros_pkg::msg::DogState &data) {
    header = data.header;
    dog_state = data.dog_state;
    return *this;
  }

  operator vmr_ros_pkg::msg::DogState() const {
    vmr_ros_pkg::msg::DogState data;
    data.header = header;
    data.dog_state = dog_state;
    return data;
  }
};



struct Exception{
  int32_t module;
  int32_t code;
  MSGPACK_DEFINE(module, code)
  Exception() {}
  Exception(const vmr_ros_pkg::msg::Exception &data) {
    module = data.module;
    code = data.code;
  }
  
  Exception &operator=(const vmr_ros_pkg::msg::Exception &data) {
    module = data.module;
    code = data.code;
    return *this;
  }

  operator vmr_ros_pkg::msg::Exception() const {
    vmr_ros_pkg::msg::Exception data;
    data.module=module;
    data.code=code;
    return data;
  }
};

struct RobotException{
  Header header;
  std::vector<Exception> exceptions;
  MSGPACK_DEFINE(header, exceptions)

  RobotException() {}
  RobotException(const vmr_ros_pkg::msg::RobotException &data) {
    header = data.header;
    int n = data.exceptions.size();
    exceptions.resize(n);
    for(int i = 0; i < n ;++i){
      exceptions[i] = data.exceptions[i];
    }
  }
  
  RobotException &operator=(const vmr_ros_pkg::msg::RobotException &data) {
    header = data.header;
    int n = data.exceptions.size();
    exceptions.resize(n);
    for(int i = 0; i < n ;++i){
      exceptions[i] = data.exceptions[i];
    }
    return *this;
  }

  operator vmr_ros_pkg::msg::RobotException() const {
    vmr_ros_pkg::msg::RobotException data;
    data.header = header;
    int n = exceptions.size();
    data.exceptions.resize(n);
    for(int i = 0; i < n ;++i){
      data.exceptions[i] = exceptions[i];
    }
    return data;
  }
};


struct TaskResult{
  int task_flag{0};
  int task_result{0};
  std::string task_id;
  MSGPACK_DEFINE(task_flag, task_result,task_id)
};


#endif  