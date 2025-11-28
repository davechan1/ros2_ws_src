cmake_minimum_required(VERSION 3.5)

project(rviz_custom_plugins)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

# Tests currently only run on OS X @ OSRF jenkins
# Enable on Linux by providing a display, enable on Windows via EnableDisplayTests=True
option(EnableDisplayTests "EnableDisplayTests")
set(DisplayTests "False" CACHE STRING "DisplayTestsVariable")

if(DEFINED ENV{DISPLAY})
  set(DISPLAYPRESENT TRUE)
endif()

if(APPLE OR DISPLAYPRESENT OR EnableDisplayTests STREQUAL "True")
  message(STATUS "Enabling tests requiring a display")
else()
  set(SKIP_DISPLAY_TESTS "SKIP_TEST")
endif()

# options and directories for visual tests (see visual_testing_framework documentation)
option(EnableVisualTests "decides whether or not to enable the tests")

add_definitions(-D_BUILD_DIR_PATH="${CMAKE_CURRENT_BINARY_DIR}")
add_definitions(-D_SRC_DIR_PATH="${CMAKE_CURRENT_SOURCE_DIR}")

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test_images)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test/reference_images)

# Run visual tests only if "EnableVisualTests=True"
if(EnableVisualTests STREQUAL "True")
  message(STATUS "Enabling visual tests")
else()
  set(SKIP_VISUAL_TESTS "SKIP_TEST")
endif()

if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

# We specifically don't turn on CMAKE_AUTOMOC, since it generates one huge
# mocs_compilation.cpp file that takes a lot of memory to compile.  Instead
# we create individual moc files that can be compiled separately.

find_package(ament_cmake_ros REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

find_package(geometry_msgs REQUIRED)

# find_package(gz_math_vendor REQUIRED)
# find_package(gz-math REQUIRED)

find_package(image_transport REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(point_cloud_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(resource_retriever REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(visualization_msgs REQUIRED)

set(rviz_custom_plugins_headers_to_moc
  include/rviz_custom_plugins/tools/goal_pose/goal_tool.hpp
)

foreach(header "${rviz_custom_plugins_headers_to_moc}")
  qt5_wrap_cpp(rviz_custom_plugins_moc_files "${header}")
endforeach()

set(rviz_custom_plugins_source_files
  src/rviz_custom_plugins/tools/goal_pose/goal_tool.cpp
  src/rviz_custom_plugins/tools/pose/pose_tool.cpp
)

add_library(rviz_custom_plugins SHARED
  ${rviz_custom_plugins_moc_files}
  ${rviz_custom_plugins_source_files}
)

target_include_directories(rviz_custom_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(rviz_custom_plugins PUBLIC
  ${geometry_msgs_TARGETS}
  image_transport::image_transport
  interactive_markers::interactive_markers
  laser_geometry::laser_geometry
  ${map_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  point_cloud_transport::point_cloud_transport
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  rviz_rendering::rviz_rendering
  ${sensor_msgs_TARGETS}
  tf2::tf2
  ${tf2_geometry_msgs_TARGETS}
  tf2_ros::tf2_ros
  urdf::urdf
  ${visualization_msgs_TARGETS}
  Qt5::Widgets
)

# target_link_libraries(rviz_custom_plugins PRIVATE
#   gz-math::core
#   resource_retriever::resource_retriever
# )

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(rviz_custom_plugins PRIVATE "rviz_custom_plugins_BUILDING_LIBRARY")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(rviz_custom_plugins HAS_LIBRARY_TARGET)

ament_export_dependencies(
  geometry_msgs
  image_transport
  interactive_markers
  laser_geometry
  map_msgs
  nav_msgs
  point_cloud_transport
  rclcpp
  rviz_common
  rviz_ogre_vendor
  rviz_rendering
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  visualization_msgs
)

install(
  TARGETS rviz_custom_plugins
  EXPORT rviz_custom_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

ament_package(
  CONFIG_EXTRAS "rviz_custom_plugins-extras.cmake"
)

if(TEST cpplint)
  set_tests_properties(cpplint PROPERTIES TIMEOUT 180)
endif()
