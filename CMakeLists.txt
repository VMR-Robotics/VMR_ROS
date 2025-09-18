cmake_minimum_required(VERSION 3.8)
project(vmr_ros_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(
        "include/rest_rpc/thirdparty/msgpack-c/include"
        "include/rest_rpc/thirdparty/asio"
        "include/rest_rpc"
        )



# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/Light.msg"
  "msg/LiftState.msg"
  "msg/Exception.msg"
  "msg/ServoState.msg"
  "srv/LightCtrl.srv"
  "srv/LiftCtrl.srv"
  "srv/TaskResult.srv"
  "msg/RobotException.msg"
  "msg/DogState.msg"
  DEPENDENCIES geometry_msgs 
)

add_executable(lx_ros_interface src/lx_ros.cpp)
ament_target_dependencies(lx_ros_interface
  rclcpp std_msgs nav_msgs sensor_msgs geometry_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target vmr_ros_pkg "rosidl_typesupport_cpp")
target_link_libraries(lx_ros_interface "${cpp_typesupport_target}")

target_include_directories(lx_ros_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


install(TARGETS lx_ros_interface
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
