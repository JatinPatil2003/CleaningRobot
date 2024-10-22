cmake_minimum_required(VERSION 3.8)
project(naviclean_coverage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp_action REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Define the executable
add_executable(coverage_planner_node
  src/controller.cpp
  src/map_loader.cpp
  src/coverage_planner.cpp
  src/coverage_planner_node.cpp
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Link libraries
target_link_libraries(coverage_planner_node
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  rclcpp::rclcpp
)

# Add dependencies to ensure correct linking
ament_target_dependencies(coverage_planner_node
  rclcpp
  nav_msgs
  nav2_msgs
  OpenCV
  yaml-cpp
  rclcpp_action
)

# Install the executable
install(TARGETS coverage_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
