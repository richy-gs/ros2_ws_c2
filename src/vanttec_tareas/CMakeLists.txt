cmake_minimum_required(VERSION 3.8)
project(vanttec_tareas)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(my_task2 src/my_task2.cpp)
ament_target_dependencies(my_task2
    rclcpp
    geometry_msgs
    sensor_msgs
)

add_executable(differential_drive_node src/differential_drive_node.cpp)
ament_target_dependencies(differential_drive_node
    rclcpp
    sensor_msgs
    geometry_msgs
)

install( TARGETS
  my_task2
  differential_drive_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
