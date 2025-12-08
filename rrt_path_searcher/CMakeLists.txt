cmake_minimum_required(VERSION 3.8)
project(rrt_path_searcher)

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

find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(ompl REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

###################### Include directories ######################
include_directories(
    include
    /opt/ros/jazzy/include/ompl-1.7
    SYSTEM 
    third_party
    ${Eigen3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

###################### Add executable ######################
add_executable(demo_rrt_node
    src/demo_rrt_node.cpp
    src/rrt_graph_searcher.cpp
)
ament_target_dependencies(demo_rrt_node
    rclcpp
    std_msgs
    nav_msgs
    visualization_msgs
    pcl_conversions
    PCL
    Eigen3
    ompl
)
target_link_libraries(demo_rrt_node
  /usr/lib/x86_64-linux-gnu/libompl.so.1.7.0
)

###################### Install targets ######################
install(TARGETS demo_rrt_node
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