cmake_minimum_required(VERSION 3.0.2)
project(adas_hal)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs
)

catkin_install_python(PROGRAMS
  scripts/sensor_bridge_carla.py
  scripts/sensor_bridge_real.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
