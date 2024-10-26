cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# We are telling to the compiler install the directory
# launch inside the my_robot_bringup folder
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# SET UP FOR LAUNCH FILE
# - Just create a new package with a launch folder
# - Install that launch folder from CMakeLists  .txt

ament_package()
