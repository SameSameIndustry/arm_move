cmake_minimum_required(VERSION 3.8)
project(arm_move)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/arm_trajectory.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_target_dependencies(${PROJECT_NAME} rclcpp)
rclcpp_components_register_nodes(${PROJECT_NAME}
  "arm_move::ArmTrajectory"
)


ament_auto_package()
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME})