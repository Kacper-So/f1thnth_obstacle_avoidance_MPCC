cmake_minimum_required(VERSION 3.14)
project(obstacle_avoidance_MPCC)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/obstacle_avoidance_MPCC/obstacle_avoidance_MPCC.hpp
  include/obstacle_avoidance_MPCC/visibility_control.hpp
  src/obstacle_avoidance_MPCC.cpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  include/obstacle_avoidance_MPCC/obstacle_avoidance_MPCC_node.hpp
  src/obstacle_avoidance_MPCC_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "obstacle_avoidance_MPCC::ObstacleAvoidanceMpccNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_obstacle_avoidance_MPCC
    test/test_obstacle_avoidance_MPCC.cpp
  )
  ament_target_dependencies(test_obstacle_avoidance_MPCC ${TEST_SOURCES})
  target_link_libraries(test_obstacle_avoidance_MPCC ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
