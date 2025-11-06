cmake_minimum_required(VERSION 3.8)
project(pioneer_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectionReport.msg"
  "msg/TerritoryAssignment.msg"
  "msg/MissionStatus.msg"
  "msg/ConsensusMessage.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
)

# Install message files
install(FILES
  msg/DetectionReport.msg
  msg/TerritoryAssignment.msg
  msg/MissionStatus.msg
  msg/ConsensusMessage.msg
  DESTINATION share/${PROJECT_NAME}/msg
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()





