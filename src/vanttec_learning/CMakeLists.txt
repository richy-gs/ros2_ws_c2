cmake_minimum_required(VERSION 3.8)
project(vanttec_learning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(turtle_tf2_publisher_node src/turtle_tf2_publisher_node.cpp)
ament_target_dependencies(turtle_tf2_publisher_node rclcpp std_msgs geometry_msgs visualization_msgs tf2_ros tf2 turtlesim)
target_compile_features(turtle_tf2_publisher_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(joint_twist_publisher_node src/joint_twist_publisher_node.cpp)
ament_target_dependencies(joint_twist_publisher_node rclcpp std_msgs geometry_msgs visualization_msgs  sensor_msgs)
target_compile_features(joint_twist_publisher_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(robot_state_publisher_node src/robot_state_publisher_node.cpp)
ament_target_dependencies(robot_state_publisher_node rclcpp std_msgs geometry_msgs visualization_msgs tf2_ros tf2 sensor_msgs)
target_compile_features(robot_state_publisher_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(turtle_follower_node src/turtle_follower_node.cpp)
ament_target_dependencies(turtle_follower_node rclcpp std_msgs geometry_msgs visualization_msgs tf2_ros tf2 tf2_geometry_msgs sensor_msgs)
target_compile_features(turtle_follower_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
  # usv_master
  turtle_tf2_publisher_node
  joint_twist_publisher_node
  robot_state_publisher_node
  turtle_follower_node
  DESTINATION lib/${PROJECT_NAME})

# Resources
install(DIRECTORY
  launch
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME})

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

ament_package()