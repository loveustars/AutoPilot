#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CARLA Sensor Bridge - Hardware Abstraction Layer for Simulation
CARLA传感器桥接 - 仿真硬件抽象层

Converts CARLA-specific sensor topics to unified ADAS topics.
This allows the perception stack to use the same interface
regardless of whether running in simulation or on real hardware.

CARLA Topics (Input):
    /carla/ego_vehicle/lidar -> /adas/lidar
    /carla/ego_vehicle/rgb_front/image -> /adas/camera/image
    /carla/ego_vehicle/odometry -> /adas/ego_state
    
Unified Topics (Output):
    /adas/lidar (sensor_msgs/PointCloud2)
    /adas/camera/image (sensor_msgs/Image)
    /adas/ego_state (geometry_msgs/TwistStamped)
"""

import rospy
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


class CarlaSensorBridge:
    def __init__(self):
        rospy.init_node('sensor_bridge_carla', anonymous=False)
        
        # Parameters - CARLA topic names
        self.lidar_topic = rospy.get_param('~carla_lidar_topic', '/carla/ego_vehicle/lidar')
        self.camera_topic = rospy.get_param('~carla_camera_topic', '/carla/ego_vehicle/rgb_front/image')
        self.odom_topic = rospy.get_param('~carla_odom_topic', '/carla/ego_vehicle/odometry')
        
        # Publishers - Unified interface
        self.lidar_pub = rospy.Publisher('/adas/lidar', PointCloud2, queue_size=1)
        self.camera_pub = rospy.Publisher('/adas/camera/image', Image, queue_size=1)
        self.ego_pub = rospy.Publisher('/adas/ego_state', TwistStamped, queue_size=1)
        
        # Subscribers - CARLA specific
        self.lidar_sub = rospy.Subscriber(
            self.lidar_topic, PointCloud2, self.lidar_callback, queue_size=1
        )
        self.camera_sub = rospy.Subscriber(
            self.camera_topic, Image, self.camera_callback, queue_size=1
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self.odom_callback, queue_size=1
        )
        
        rospy.loginfo("CARLA Sensor Bridge initialized")
        rospy.loginfo(f"  LiDAR: {self.lidar_topic} -> /adas/lidar")
        rospy.loginfo(f"  Camera: {self.camera_topic} -> /adas/camera/image")
        rospy.loginfo(f"  Odometry: {self.odom_topic} -> /adas/ego_state")
    
    def lidar_callback(self, msg: PointCloud2):
        """Forward LiDAR data to unified topic"""
        # Optionally transform frame_id here
        msg.header.frame_id = "adas_lidar"
        self.lidar_pub.publish(msg)
    
    def camera_callback(self, msg: Image):
        """Forward camera image to unified topic"""
        msg.header.frame_id = "adas_camera"
        self.camera_pub.publish(msg)
    
    def odom_callback(self, msg: Odometry):
        """Convert odometry to ego state"""
        ego_state = TwistStamped()
        ego_state.header = msg.header
        ego_state.header.frame_id = "adas_base_link"
        ego_state.twist = msg.twist.twist
        
        self.ego_pub.publish(ego_state)
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = CarlaSensorBridge()
        node.run()
    except rospy.ROSInterruptException:
        pass
