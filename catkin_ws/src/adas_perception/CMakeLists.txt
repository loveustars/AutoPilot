cmake_minimum_required(VERSION 3.0.2)
project(adas_perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# C++ PointCloud Filter Node (Phase 2 optimization)
add_executable(pointcloud_filter_node src/pointcloud_filter.cpp)
target_link_libraries(pointcloud_filter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pointcloud_filter_node ${catkin_EXPORTED_TARGETS})

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/lidar_processor.py
  scripts/camera_detector.py
  scripts/obstacle_tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
