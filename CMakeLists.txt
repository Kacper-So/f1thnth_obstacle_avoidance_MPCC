cmake_minimum_required(VERSION 3.5)
project(traj_selector)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(freespace_planning_algorithms REQUIRED)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${autoware_auto_vehicle_msgs_INCLUDE_DIRS}
)

add_executable(traj_selector src/traj_selector.cpp)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  autoware_auto_planning_msgs
  autoware_auto_control_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  tf2
  OpenCV
  cv_bridge
  autoware_auto_vehicle_msgs
  tf2_ros
  tf2_geometry_msgs
  freespace_planning_algorithms
)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
