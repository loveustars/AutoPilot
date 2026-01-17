#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Vehicle Controller - Unified control interface
统一车辆控制接口

Converts high-level commands to platform-specific actuation.
Supports both CARLA simulation and real vehicle (via parameter switch).

Subscribed Topics:
    /adas/safe_command (geometry_msgs/AccelStamped) - from safety monitor
    
Published Topics:
    /carla/ego_vehicle/vehicle_control_cmd (carla_msgs/CarlaEgoVehicleControl) - for CARLA
    OR
    /vehicle/brake_cmd (platform specific) - for real vehicle
"""

import rospy
import numpy as np
from geometry_msgs.msg import AccelStamped

# Try to import CARLA messages
try:
    from carla_msgs.msg import CarlaEgoVehicleControl
    CARLA_AVAILABLE = True
except ImportError:
    CARLA_AVAILABLE = False
    rospy.logwarn("carla_msgs not found. Make sure to source carla-ros-bridge workspace.")
    rospy.logwarn("Run: source ~/carla-ros-bridge/catkin_ws/devel/setup.bash")


class VehicleController:
    def __init__(self):
        rospy.init_node('vehicle_controller', anonymous=False)
        
        # Parameters
        self.mode = rospy.get_param('~mode', 'simulation')  # 'simulation' or 'real'
        self.max_decel = rospy.get_param('~max_deceleration', 10.0)
        self.brake_gain = rospy.get_param('~brake_gain', 0.1)  # decel to brake pedal
        
        # Current state
        self.current_throttle = 0.0
        self.current_brake = 0.0
        self.current_steer = 0.0
        
        # Publishers
        if self.mode == 'simulation' and CARLA_AVAILABLE:
            self.control_pub = rospy.Publisher(
                '/carla/ego_vehicle/vehicle_control_cmd', 
                CarlaEgoVehicleControl, 
                queue_size=1
            )
            rospy.loginfo("Vehicle Controller: CARLA simulation mode")
            rospy.loginfo("  Publishing to: /carla/ego_vehicle/vehicle_control_cmd")
        else:
            # Real vehicle placeholder
            self.control_pub = rospy.Publisher(
                '/vehicle/brake_cmd',
                AccelStamped,  # Placeholder - replace with actual msg
                queue_size=1
            )
            rospy.loginfo("Vehicle Controller: Real vehicle mode (placeholder)")
        
        # Subscribers
        self.cmd_sub = rospy.Subscriber(
            '/adas/safe_command', AccelStamped, self.command_callback, queue_size=1
        )
        
        # Control loop timer
        self.control_rate = rospy.get_param('~control_rate', 50.0)
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate),
            self.control_loop
        )
        
        rospy.loginfo("Vehicle Controller initialized")
    
    def command_callback(self, msg: AccelStamped):
        """Process deceleration command"""
        decel = -msg.accel.linear.x  # Positive decel from negative accel
        decel = np.clip(decel, 0.0, self.max_decel)
        
        # Convert deceleration to brake pedal position [0, 1]
        self.current_brake = np.clip(decel * self.brake_gain, 0.0, 1.0)
        
        # Cut throttle when braking
        if self.current_brake > 0.01:
            self.current_throttle = 0.0
    
    def control_loop(self, event):
        """Periodic control output"""
        if self.mode == 'simulation' and CARLA_AVAILABLE:
            self._publish_carla_control()
        else:
            self._publish_real_control()
    
    def _publish_carla_control(self):
        """Publish control to CARLA"""
        msg = CarlaEgoVehicleControl()
        msg.header.stamp = rospy.Time.now()
        msg.throttle = self.current_throttle
        msg.brake = self.current_brake
        msg.steer = self.current_steer
        msg.hand_brake = False
        msg.reverse = False
        msg.manual_gear_shift = False
        
        self.control_pub.publish(msg)
    
    def _publish_real_control(self):
        """Publish control to real vehicle interface"""
        # TODO: Implement real vehicle control protocol
        # This is a placeholder - actual implementation depends on vehicle CAN interface
        msg = AccelStamped()
        msg.header.stamp = rospy.Time.now()
        msg.accel.linear.x = -self.current_brake * self.max_decel
        self.control_pub.publish(msg)
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = VehicleController()
        node.run()
    except rospy.ROSInterruptException:
        pass
