cmake_minimum_required(VERSION 3.8)
project(rrtstar_octomap_planner)

# Force C++17 for ROS2 Jazzy compatibility
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(frontier_extraction_msgs REQUIRED)
find_package(frontier_extraction_srvs REQUIRED)

# Manual include and library setup for OMPL
set(OMPL_INCLUDE_DIRS 
  "/usr/include/ompl-1.5"
  "/usr/include/eigen3"
)

set(OMPL_LIBRARIES 
  "ompl"
  "boost_serialization"
  "boost_filesystem" 
  "boost_system"
  "ode"
)

message(STATUS "Using manual dependency configuration")
message(STATUS "OMPL includes: ${OMPL_INCLUDE_DIRS}")
message(STATUS "OMPL libraries: ${OMPL_LIBRARIES}")

# Include directories
include_directories(
  include
  ${OMPL_INCLUDE_DIRS}
)

# Add C++14 compatibility for OMPL (but keep overall C++17)
add_compile_options(-Wno-c++14-compat)

# ========== ORIGINAL PLANNER NODE ==========
add_executable(planner_node src/planner_node.cpp)

# Use ament_target_dependencies for ROS2 packages
ament_target_dependencies(planner_node
  rclcpp
  px4_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  octomap_msgs
)

# Link system libraries
target_link_libraries(planner_node
  ${OMPL_LIBRARIES}
  octomap
  octomath
)

# ========== AUTONOMOUS PLANNER NODE ==========
add_executable(autonomous_planner_node 
  src/autonomous_planner_node.cpp
)

# Add dependencies for the new node
ament_target_dependencies(autonomous_planner_node
  rclcpp
  px4_msgs
  geometry_msgs
  nav_msgs
  octomap_msgs
  visualization_msgs
  frontier_extraction_msgs
  frontier_extraction_srvs
)

# Link system libraries for autonomous planner
target_link_libraries(autonomous_planner_node
  ${OMPL_LIBRARIES}
  octomap
  octomath
)

# ========== INSTALL TARGETS ==========
install(TARGETS 
  planner_node
  autonomous_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# ========== EXPORT DEPENDENCIES ==========
ament_export_dependencies(
  rclcpp
  px4_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  octomap_msgs
  visualization_msgs
  frontier_extraction_msgs
  frontier_extraction_srvs
)

# ========== FINAL PACKAGE COMMAND ==========
ament_package()