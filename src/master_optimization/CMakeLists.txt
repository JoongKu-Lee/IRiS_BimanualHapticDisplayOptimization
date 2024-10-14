cmake_minimum_required(VERSION 3.8)
project(master_optimization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(ament_index_cpp REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(kdl_parser REQUIRED)


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

install (DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(DIRECTORY model DESTINATION share/${PROJECT_NAME})

install(DIRECTORY data DESTINATION share/${PROJECT_NAME})

add_library(optimization SHARED
  src/human_upper_body_human_left_temp_ikfast_solver.cpp
  src/human_upper_body_human_right_temp_ikfast_solver.cpp
  src/human_tworobot_leftpanda_ikfast_solver.cpp
)

add_executable(calculate_score src/calculate_score.cpp)
ament_target_dependencies(calculate_score rclcpp std_msgs sensor_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface ament_index_cpp Eigen3 eigen3_cmake_module kdl_parser) 

install (TARGETS calculate_score DESTINATION lib/${PROJECT_NAME})

ament_package()
