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
#include "vmr_ros_pkg/srv/task_result.hpp"

#include "vmr_ros_pkg/msg/lift_state.hpp"
#include "vmr_ros_pkg/msg/dog_state.hpp"

// #include "vmr_motor_interfaces/msg/motor_status.hpp"

// 对应原 RobotState.msg
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

#include "rest_rpc.hpp"
#include "vmr_data.h"
#include "vmr_ros_pkg/msg/exception.hpp"

#include "vmr_ros_pkg/msg/robot_exception.hpp"
#include "vmr_ros_pkg/msg/dog_state.hpp"

using namespace rest_rpc;
using namespace rpc_service;
class ListenerNode : public rclcpp::Node {
public:
  ListenerNode();

private:
  // sub
  std::string frame_id = "vmr_robot";
  rpc_client cli;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser2_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      sub_locate_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_twist_;

  rclcpp::Subscription<vmr_ros_pkg::msg::Light>::SharedPtr sub_light_;

  // pub
  rclcpp::Publisher<vmr_ros_pkg::msg::RobotState>::SharedPtr pub_state_;
  rclcpp::Publisher<vmr_ros_pkg::msg::Light>::SharedPtr pub_light_state_;
  rclcpp::Publisher<vmr_ros_pkg::msg::LiftState>::SharedPtr pub_lift_state_;
  rclcpp::Publisher<vmr_ros_pkg::msg::DogState>::SharedPtr pub_dog_state_;

  //rclcpp::Publisher<vmr_ros_pkg::msg::MotorStatus>::SharedPtr
  //    pub_lift_join_state_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_state_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr
      pub_battery_state_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_state_;

  rclcpp::Publisher<vmr_ros_pkg::msg::ServoState>::SharedPtr pub_header_state_1;
  rclcpp::Publisher<vmr_ros_pkg::msg::ServoState>::SharedPtr pub_header_state_2;

  rclcpp::Publisher<vmr_ros_pkg::msg::RobotException>::SharedPtr
      pub_robot_exception_state;
  // service
  rclcpp::Service<vmr_ros_pkg::srv::LiftCtrl>::SharedPtr server_lift_ctrl;
  rclcpp::Service<vmr_ros_pkg::srv::LightCtrl>::SharedPtr server_light_ctrl;
  rclcpp::Service<vmr_ros_pkg::srv::TaskResult>::SharedPtr server_task_result;
  // rclcpp::Service<vmr_ros_pkg::srv::LightCtrl>::SharedPtr server_task_ctrl;
  /* ---------- 回调函数 ---------- */
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                     int location);

  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr data);

  void lightCallback(const vmr_ros_pkg::msg::Light::SharedPtr data);

  void header_LR_Callback(const vmr_ros_pkg::msg::ServoState::SharedPtr data);

  void header_UD_Callback(const vmr_ros_pkg::msg::ServoState::SharedPtr data);

  void processPose(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void bo_callback(string_view data);

  void light_callback(string_view data);
  void imu_callback(string_view data);

  void battery_callback(string_view data);

  void odom_callback(string_view data);
  void exception_state_callback(string_view data);
  void lift_state_callback(string_view data);

  void dog_state_callback(string_view data);
  void header_1_callback(string_view data);

  void header_2_callback(string_view data);

  void lift_handle_service(
      const std::shared_ptr<vmr_ros_pkg::srv::LiftCtrl::Request> request,
      std::shared_ptr<vmr_ros_pkg::srv::LiftCtrl::Response> response);

  void light_handle_service(
      vmr_ros_pkg::srv::LightCtrl::Request::SharedPtr request,
      vmr_ros_pkg::srv::LightCtrl::Response::SharedPtr response);

  void task_result_service(
      vmr_ros_pkg::srv::TaskResult::Request::SharedPtr request,
      vmr_ros_pkg::srv::TaskResult::Response::SharedPtr response);
};
