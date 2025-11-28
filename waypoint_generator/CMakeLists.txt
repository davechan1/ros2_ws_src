cmake_minimum_required(VERSION 3.8)
project(waypoint_generator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###################### Find dependencies ######################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(turtlesim REQUIRED)

find_package(Eigen3 REQUIRED)

###################### Include directories ######################
include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
)

###################### Add executable ######################
add_executable(static_turtle_tf2_broadcaster src/static_turtle_tf2_broadcaster.cpp)
ament_target_dependencies(static_turtle_tf2_broadcaster
    rclcpp
    std_msgs
    geometry_msgs
    tf2
    tf2_ros
)

add_executable(waypoint_generator src/waypoint_generator.cpp)
ament_target_dependencies(waypoint_generator
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    Eigen3
)

###################### Install targets ######################
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS static_turtle_tf2_broadcaster waypoint_generator
   DESTINATION lib/${PROJECT_NAME}
)

ament_package()
