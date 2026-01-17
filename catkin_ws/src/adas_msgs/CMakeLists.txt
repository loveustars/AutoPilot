cmake_minimum_required(VERSION 3.0.2)
project(adas_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
)

# Message files
add_message_files(
  FILES
  Obstacle.msg
  ObstacleArray.msg
  TTCInfo.msg
  AEBState.msg
  SafetyStatus.msg
)

# Service files
add_service_files(
  FILES
  SetAEBMode.srv
)

# Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
