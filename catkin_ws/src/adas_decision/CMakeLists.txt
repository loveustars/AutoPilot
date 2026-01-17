cmake_minimum_required(VERSION 3.0.2)
project(adas_decision)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_runtime
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs message_runtime
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/ttc_calculator.py
  scripts/aeb_fsm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch and config
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
