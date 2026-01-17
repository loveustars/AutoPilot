#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Vehicle Controller - Unified control interface with Emergency Override
统一车辆控制接口（支持紧急覆盖）

Converts high-level commands to platform-specific actuation.
Supports both CARLA simulation and real vehicle (via parameter switch).
In emergency situations (FULL_BRAKE), overrides user manual control by
disabling manual_override mode in CARLA.

CARLA控制优先级架构:
  - Normal模式: 发布到 vehicle_control_cmd (当manual_override=False)
  - Manual模式: 发布到 vehicle_control_cmd_manual (当manual_override=True)
  
AEB通过发布False到vehicle_control_manual_override来获取控制权。

Subscribed Topics:
    /adas/safe_command (geometry_msgs/AccelStamped) - from safety monitor
    /adas/aeb_state (adas_msgs/AEBState) - for emergency override detection
    
Published Topics:
    /carla/ego_vehicle/vehicle_control_cmd (carla_msgs/CarlaEgoVehicleControl)
    /carla/ego_vehicle/vehicle_control_manual_override (std_msgs/Bool) - control mode switch
"""

import rospy
import numpy as np
from geometry_msgs.msg import AccelStamped
from std_msgs.msg import Bool

from adas_msgs.msg import AEBState

# Try to import CARLA messages
try:
    from carla_msgs.msg import CarlaEgoVehicleControl
    CARLA_AVAILABLE = True
except ImportError:
    CARLA_AVAILABLE = False
    rospy.logwarn("carla_msgs not found. Make sure to source carla-ros-bridge workspace.")
    rospy.logwarn("Run: source ~/carla-ros-bridge/catkin_ws/devel/setup.bash")


class VehicleController:
    # AEB State constants
    AEB_INACTIVE = 0
    AEB_WARNING = 1
    AEB_PARTIAL_BRAKE = 2
    AEB_FULL_BRAKE = 3
    
    def __init__(self):
        rospy.init_node('vehicle_controller', anonymous=False)
        
        # Parameters
        self.mode = rospy.get_param('~mode', 'simulation')  # 'simulation' or 'real'
        self.max_decel = rospy.get_param('~max_deceleration', 10.0)
        self.brake_gain = rospy.get_param('~brake_gain', 0.1)  # decel to brake pedal
        self.control_rate = rospy.get_param('~control_rate', 50.0)
        self.role_name = rospy.get_param('~role_name', 'ego_vehicle')
        
        # Emergency override settings
        self.emergency_override_enabled = rospy.get_param('~emergency_override', True)
        
        # Current state
        self.current_throttle = 0.0
        self.current_brake = 0.0
        self.current_steer = 0.0
        self.aeb_state = self.AEB_INACTIVE
        self.emergency_active = False
        self.manual_override_was_enabled = True  # Track previous state to restore
        
        # Publishers
        if self.mode == 'simulation' and CARLA_AVAILABLE:
            # Control command publisher (Normal mode)
            self.control_pub = rospy.Publisher(
                f'/carla/{self.role_name}/vehicle_control_cmd', 
                CarlaEgoVehicleControl, 
                queue_size=1
            )
            # Manual override switch publisher (latched for persistence)
            self.manual_override_pub = rospy.Publisher(
                f'/carla/{self.role_name}/vehicle_control_manual_override',
                Bool,
                queue_size=1,
                latch=True  # Ensures late subscribers get the last value
            )
            rospy.loginfo("Vehicle Controller: CARLA simulation mode")
            rospy.loginfo(f"  Control topic: /carla/{self.role_name}/vehicle_control_cmd")
            rospy.loginfo(f"  Override topic: /carla/{self.role_name}/vehicle_control_manual_override")
        else:
            # Real vehicle placeholder
            self.control_pub = rospy.Publisher(
                '/vehicle/brake_cmd',
                AccelStamped,  # Placeholder - replace with actual msg
                queue_size=1
            )
            self.manual_override_pub = None
            rospy.loginfo("Vehicle Controller: Real vehicle mode (placeholder)")
        
        # Subscribers
        self.cmd_sub = rospy.Subscriber(
            '/adas/safe_command', AccelStamped, self.command_callback, queue_size=1
        )
        
        # Subscribe to AEB state for emergency override detection
        self.aeb_state_sub = rospy.Subscriber(
            '/adas/aeb_state', AEBState, self.aeb_state_callback, queue_size=1
        )
        
        # Control loop timer
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate),
            self.control_loop
        )
        
        rospy.loginfo("Vehicle Controller initialized")
        rospy.loginfo(f"  Emergency override: {'enabled' if self.emergency_override_enabled else 'disabled'}")
    
    def aeb_state_callback(self, msg: AEBState):
        """Track AEB state for emergency override"""
        old_state = self.aeb_state
        self.aeb_state = msg.state
        
        # Check for emergency condition (PARTIAL_BRAKE or FULL_BRAKE)
        if self.emergency_override_enabled and msg.state >= self.AEB_PARTIAL_BRAKE:
            if not self.emergency_active:
                self.emergency_active = True
                # Disable manual override to take control from manual_control.py
                if self.manual_override_pub is not None:
                    self.manual_override_pub.publish(Bool(data=False))
                    rospy.logwarn("EMERGENCY OVERRIDE: Disabling manual control, AEB taking over")
        elif self.emergency_active and msg.state == self.AEB_INACTIVE:
            self.emergency_active = False
            # Restore manual control after emergency ends
            if self.manual_override_pub is not None:
                self.manual_override_pub.publish(Bool(data=True))
                rospy.loginfo("Emergency override released, manual control restored")
    
    def command_callback(self, msg: AccelStamped):
        """Process deceleration command"""
        decel = -msg.accel.linear.x  # Positive decel from negative accel
        decel = np.clip(decel, 0.0, self.max_decel)
        
        # Convert deceleration to brake pedal position [0, 1]
        self.current_brake = np.clip(decel * self.brake_gain, 0.0, 1.0)
        
        # In emergency states, force appropriate braking
        if self.aeb_state == self.AEB_FULL_BRAKE:
            self.current_brake = 1.0  # Full brake pedal
            self.current_throttle = 0.0
        elif self.aeb_state == self.AEB_PARTIAL_BRAKE:
            self.current_brake = max(self.current_brake, 0.5)  # At least 50% brake
            self.current_throttle = 0.0
        
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
