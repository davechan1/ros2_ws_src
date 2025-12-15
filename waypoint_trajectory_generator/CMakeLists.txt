cmake_minimum_required(VERSION 3.8)
project(waypoint_trajectory_generator)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0144)
  cmake_policy(SET CMP0144 NEW)
endif()
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 OLD)
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
find_package(visualization_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen)

###################### Include directories ######################
include_directories(
    include
  	${catkin_INCLUDE_DIRS}
  	${Eigen3_INCLUDE_DIRS} 
)

###################### Add executable ######################
add_executable(trajectory_generator_node 
	src/trajectory_generator_node.cpp
	src/trajectory_generator_waypoint.cpp
)
ament_target_dependencies(trajectory_generator_node
    rclcpp
    std_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    Eigen3
)
target_link_libraries(trajectory_generator_node OsqpEigen::OsqpEigen)

###################### Install targets ######################
install(TARGETS trajectory_generator_node
   DESTINATION lib/${PROJECT_NAME}
)
# Install headers
install(
    DIRECTORY include/
    DESTINATION include/
)
# Install the launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Package export
ament_package()
