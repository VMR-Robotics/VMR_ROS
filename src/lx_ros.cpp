#include <chrono>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vmr_ros_pkg/msg/robot_state.hpp" // 对应原 RobotState.msg
#include "vmr_ros_pkg/srv/lift_ctrl.hpp"
#include "vmr_ros_pkg/srv/light_ctrl.hpp"
#include "vmr_ros_pkg/msg/dog_state.hpp"

// 对应原 RobotState.msg
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

#include "lx_ros.h"
#include "rest_rpc.hpp"
#include "rpc_data.h"
using namespace rest_rpc;
using namespace rpc_service;

ListenerNode::ListenerNode() : Node("lx_ros_interface") {
  // 参数声明
  this->declare_parameter<std::string>("ip", "127.0.0.1");
  this->declare_parameter<int>("host", 9000);
  this->declare_parameter<std::string>("odom", "");
  this->declare_parameter<std::string>("laser_1", "");
  this->declare_parameter<std::string>("laser_2", "");
  this->declare_parameter<std::string>("locate", "");
  this->declare_parameter<std::string>("twist", "");
  this->declare_parameter<std::string>("light", "");
  this->declare_parameter<std::string>("frame_id", "");
  // 读取参数
  std::string ip = this->get_parameter("ip").as_string();
  int host = this->get_parameter("host").as_int();

  std::string odom_topic = this->get_parameter("odom").as_string();
  std::string laser1_topic = this->get_parameter("laser_1").as_string();
  std::string laser2_topic = this->get_parameter("laser_2").as_string();
  std::string locate_topic = this->get_parameter("locate").as_string();

  std::string twist_topic = this->get_parameter("twist").as_string();

  std::string light_topic = this->get_parameter("light").as_string();

  frame_id = this->get_parameter("frame_id").as_string();

  server_lift_ctrl = this->create_service<vmr_ros_pkg::srv::LiftCtrl>(
      "/vmr/robot_lift_ctrl",
      std::bind(&ListenerNode::lift_handle_service, this, std::placeholders::_1,
                std::placeholders::_2));
  server_light_ctrl = this->create_service<vmr_ros_pkg::srv::LightCtrl>(
      "/vmr/robot_light_ctrl",
      std::bind(&ListenerNode::light_handle_service, this,
                std::placeholders::_1, std::placeholders::_2));
  server_task_result = this->create_service<vmr_ros_pkg::srv::TaskResult>(
      "/vmr/robot_task_result",
      std::bind(&ListenerNode::task_result_service, this, std::placeholders::_1,
                std::placeholders::_2));

  while (!cli.connect(ip, host)) {
    RCLCPP_WARN(this->get_logger(), "connect fail, retry ...");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  cli.enable_auto_reconnect();
  cli.enable_auto_heartbeat();
  RCLCPP_INFO(this->get_logger(), "RPC connected");

  // 订阅 / 发布
  if (!odom_topic.empty())
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 1000,
        std::bind(&ListenerNode::odomCallback, this, std::placeholders::_1));
  if (!laser1_topic.empty())
    sub_laser1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        laser1_topic, 1000,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          laserCallback(msg, 1);
        });
  if (!laser2_topic.empty())
    sub_laser2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        laser2_topic, 1000,
        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          laserCallback(msg, 2);
        });
  if (!locate_topic.empty())
    sub_locate_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        locate_topic, 1000,
        std::bind(&ListenerNode::processPose, this, std::placeholders::_1));
  if (!twist_topic.empty())
    sub_twist_ = this->create_subscription<geometry_msgs::msg::Twist>(
        twist_topic, 1000,
        std::bind(&ListenerNode::twistCallback, this, std::placeholders::_1));
  if (!light_topic.empty())
    sub_light_ = this->create_subscription<vmr_ros_pkg::msg::Light>(
        light_topic, 1000,
        std::bind(&ListenerNode::lightCallback, this, std::placeholders::_1));

  pub_state_ = this->create_publisher<vmr_ros_pkg::msg::RobotState>(
      "/vmr/robot_loc_state", 100);

  pub_light_state_ = this->create_publisher<vmr_ros_pkg::msg::Light>(
      "/vmr/robot_light_state", 100);

  pub_imu_state_ =
      this->create_publisher<sensor_msgs::msg::Imu>("/vmr/robot_imu", 100);

  pub_battery_state_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
      "/vmr/robot_battery", 100);

  pub_odometry_state_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/vmr/robot_odometry", 100);

  pub_lift_state_ = this->create_publisher<vmr_ros_pkg::msg::LiftState>(
      "/vmr/robot_lift_state", 100);
  pub_dog_state_ = this->create_publisher<vmr_ros_pkg::msg::DogState>(
      "/vmr/robot_dog_state", 100);
  //pub_lift_join_state_ = this->create_publisher<vmr_ros_pkg::msg::MotorStatus>(
  //    "/vmr/robot_lift_join_state", 100);
  pub_robot_exception_state =
      this->create_publisher<vmr_ros_pkg::msg::RobotException>(
          "/vmr/robot_exception_state", 100);

  // RPC 反向订阅
  cli.subscribe("hontai/rob_data",
                [this](string_view data) { bo_callback(data); });

  cli.subscribe("hontai/light_state",
                [this](string_view data) { light_callback(data); });

  cli.subscribe("hontai/IMU", [this](string_view data) { imu_callback(data); });
  cli.subscribe("hontai/battery",
                [this](string_view data) { battery_callback(data); });
  cli.subscribe("hontai/odometry",
                [this](string_view data) { odom_callback(data); });
  cli.subscribe("hontai/lift_state",
                [this](string_view data) { lift_state_callback(data); });
  cli.subscribe("hontai/dog_state",
                [this](string_view data) { dog_state_callback(data); });
  cli.subscribe("hontai/rob_exception",
                [this](string_view data) { exception_state_callback(data); });
}

void ListenerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  BasicOdometry basic_odom = BasicOdometry::fromRos(*msg);
  try {
    if (cli.has_connected()) {
      cli.call<int>("listen_odometer", basic_odom);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "RPC error: %s", e.what());
  }
}

void ListenerNode::laserCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg, int location) {
  BasicLaserScan basic_scan = BasicLaserScan::fromRos(*msg);
  RCLCPP_INFO(this->get_logger(), "time : %s",
              std::to_string(basic_scan.header.timestamp_ns).c_str());
  RCLCPP_INFO(this->get_logger(), "frame_id : %s",
              basic_scan.header.frame_id.c_str());

  basic_scan.header.frame_id += "_" + std::to_string(location);
  RCLCPP_INFO(this->get_logger(), "add location frame_id : %s",
              basic_scan.header.frame_id.c_str());

  try {
    if (cli.has_connected()) {
      cli.call<int>("listen_laser", basic_scan);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "RPC error: %s", e.what());
  }
}

void ListenerNode::twistCallback(
    const geometry_msgs::msg::Twist::SharedPtr data) {
  Twist ts = *data;
  // RCLCPP_INFO(this->get_logger(), "twist time : %s",
  //             std::to_string(ts.header.timestamp_ns).c_str());
  try {
    if (cli.has_connected()) {
      cli.call<int>("control_twist", ts);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "RPC error: %s", e.what());
  }
}

void ListenerNode::lightCallback(
    const vmr_ros_pkg::msg::Light::SharedPtr data) {
  Light light = *data;
  RCLCPP_INFO(this->get_logger(), "light time : %s",
              std::to_string(light.header.timestamp_ns).c_str());
  try {
    if (cli.has_connected()) {
      cli.call<int>("control_light", light);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "RPC error: %s", e.what());
  }
}

// 控制左右头
void ListenerNode::header_LR_Callback(
    const vmr_ros_pkg::msg::ServoState::SharedPtr data) {
  ServoState light = *data;
  RCLCPP_INFO(this->get_logger(), "light time : %s",
              std::to_string(light.header.timestamp_ns).c_str());
  try {
    if (cli.has_connected()) {
      cli.call<int>("control_head_LR", light);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "RPC error: %s", e.what());
  }
}

void ListenerNode::header_UD_Callback(
    const vmr_ros_pkg::msg::ServoState::SharedPtr data) {
  ServoState light = *data;
  RCLCPP_INFO(this->get_logger(), "light time : %s",
              std::to_string(light.header.timestamp_ns).c_str());
  try {
    if (cli.has_connected()) {
      cli.call<int>("control_head_UD", light);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "RPC error: %s", e.what());
  }
}

void ListenerNode::processPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  auto data = PoseWithCovarianceStamped::fromRos(*msg);
  RCLCPP_INFO(this->get_logger(), "time : %s",
              std::to_string(data.header.timestamp_ns).c_str());
  RCLCPP_INFO(this->get_logger(), "add location frame_id : %s",
              data.header.frame_id.c_str());

  try {
    if (cli.has_connected()) {
      cli.call<int>("listen_robot_relocate", data);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "RPC error: %s", e.what());
  }
}

void ListenerNode::bo_callback(string_view data) {
  msgpack_codec codec;
  LocState p = codec.unpack<LocState>(data.data(), data.size());
  vmr_ros_pkg::msg::RobotState msg = LocState::toRos(p);

  if (pub_state_) {
    msg.header.frame_id = frame_id;
    pub_state_->publish(msg);
  }
}

void ListenerNode::light_callback(string_view data) {
  msgpack_codec codec;
  Light p = codec.unpack<Light>(data.data(), data.size());
  vmr_ros_pkg::msg::Light msg = p;

  if (pub_light_state_) {
    msg.header.frame_id = frame_id;
    pub_light_state_->publish(msg);
  }
}

void ListenerNode::exception_state_callback(string_view data) {
  msgpack_codec codec;
  RobotException p = codec.unpack<RobotException>(data.data(), data.size());
  vmr_ros_pkg::msg::RobotException msg = p;
  if (pub_robot_exception_state) {
    msg.header.frame_id = frame_id;
    pub_robot_exception_state->publish(msg);
  }
}

void ListenerNode::imu_callback(string_view data) {
  msgpack_codec codec;
  Imu p = codec.unpack<Imu>(data.data(), data.size());
  sensor_msgs::msg::Imu msg = p;

  if (pub_imu_state_) {
    msg.header.frame_id = frame_id;
    pub_imu_state_->publish(msg);
  }
}

void ListenerNode::battery_callback(string_view data) {
  msgpack_codec codec;
  BatteryState p = codec.unpack<BatteryState>(data.data(), data.size());
  sensor_msgs::msg::BatteryState msg = p;

  if (pub_battery_state_) {
    msg.header.frame_id = frame_id;
    pub_battery_state_->publish(msg);
  }
}

void ListenerNode::odom_callback(string_view data) {
  msgpack_codec codec;
  BasicOdometry p = codec.unpack<BasicOdometry>(data.data(), data.size());
  nav_msgs::msg::Odometry msg = p.toRos();

  if (pub_odometry_state_) {
    msg.header.frame_id = frame_id;
    pub_odometry_state_->publish(msg);
  }
}

void ListenerNode::lift_state_callback(string_view data) {
  msgpack_codec codec;
  LiftState p = codec.unpack<LiftState>(data.data(), data.size());
  vmr_ros_pkg::msg::LiftState msg = p;
  if (pub_lift_state_) {
    msg.header.frame_id = frame_id;
    pub_lift_state_->publish(msg);
  }
  // lx_motor_interfaces::msg::MotorStatus left_state;
  // left_state.data[0] = p.height/1000.0;
  // if (pub_lift_join_state_) {
  //     msg.header.frame_id = frame_id;
  //     lx_motor_interfaces->publish(left_state);
  // }
}
void ListenerNode::dog_state_callback(string_view data) {
  msgpack_codec codec;
  DogState p = codec.unpack<DogState>(data.data(), data.size());
  vmr_ros_pkg::msg::DogState msg = p;
  if (pub_dog_state_) {
    msg.header.frame_id = frame_id;
    pub_dog_state_->publish(msg);
  }

}
void ListenerNode::light_handle_service(
    vmr_ros_pkg::srv::LightCtrl::Request::SharedPtr request,
    vmr_ros_pkg::srv::LightCtrl::Response::SharedPtr response) {
  RCLCPP_INFO(this->get_logger(), "light ctrl: %d", request->light_state);
  try {
    if (cli.has_connected()) {
      cli.call<int>("control_light", request->light_state);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "RPC error: %s", e.what());
  }
  response->code = 200;
}
void ListenerNode::lift_handle_service(
    const std::shared_ptr<vmr_ros_pkg::srv::LiftCtrl::Request> request,
    std::shared_ptr<vmr_ros_pkg::srv::LiftCtrl::Response> response) {
  RCLCPP_INFO(this->get_logger(), "left ctrl: %d", request->height);
  try {
    if (cli.has_connected()) {
      response->task_id =
          cli.call<std::string>("control_lift", request->height);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "RPC error: %s", e.what());
  }
  if(response->task_id != ""){
    response->code = 200;
  }else{
    response->code = 400;
  }
  
}

void ListenerNode::task_result_service(
    const std::shared_ptr<vmr_ros_pkg::srv::TaskResult::Request> request,
    std::shared_ptr<vmr_ros_pkg::srv::TaskResult::Response> response) {
  try {
    if (cli.has_connected()) {
      auto task_result = cli.call<TaskResult>("task_result", request->task_id);

      response->task_id = request->task_id;
      response->task_flag = task_result.task_flag;
      response->task_result = task_result.task_result;
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "RPC error: %s", e.what());
  }
  response->code = 200;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ListenerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}