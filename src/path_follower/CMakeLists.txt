cmake_minimum_required(VERSION 3.8)
project(path_follower)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)


# Create executable
add_executable(path_follower src/path_follower.cpp)

# Specify dependencies
ament_target_dependencies(path_follower
  rclcpp
  px4_msgs
  nav_msgs
  geometry_msgs
  octomap_msgs
  octomap
)

# Install the executable
install(TARGETS
  path_follower
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()