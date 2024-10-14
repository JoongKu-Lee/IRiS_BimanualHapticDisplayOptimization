cmake_minimum_required(VERSION 3.8)
project(gen_humanik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find package for #include "moveit/robot_model_loader/robot_model_loader.h"
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(ament_index_cpp REQUIRED)

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

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

add_library(gen_humanik SHARED 
src/human_upper_body_human_left_temp_ikfast_solver.cpp
src/human_upper_body_human_right_temp_ikfast_solver.cpp)

add_executable(humanik src/analyze_human_and_savecsv.cpp)
ament_target_dependencies(humanik rclcpp std_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface sensor_msgs ament_index_cpp)


install(TARGETS
  humanik
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
