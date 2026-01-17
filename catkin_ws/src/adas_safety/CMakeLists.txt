cmake_minimum_required(VERSION 3.0.2)
project(adas_safety)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

# C++ Safety Monitor (Phase 3 - Real vehicle)
add_executable(safety_watchdog_node src/safety_watchdog.cpp)
target_link_libraries(safety_watchdog_node ${catkin_LIBRARIES})
add_dependencies(safety_watchdog_node ${catkin_EXPORTED_TARGETS})

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/safety_monitor.py
  scripts/heartbeat_watchdog.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
