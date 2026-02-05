#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AEB Finite State Machine
自动紧急制动状态机 - 直接控制CARLA刹车

States:
    INACTIVE:      TTC > 3.0s
    WARNING:       1.5s <= TTC < 3.0s (FCW alert)
    PARTIAL_BRAKE: 1.0s <= TTC < 1.5s (decel 4.0-7.0 m/s²)
    FULL_BRAKE:    TTC < 1.0s (brake=1.0, 踩死)

Subscribed Topics:
    /adas/ttc_info (adas_msgs/TTCInfo)
    
Published Topics:
    /adas/aeb_state (adas_msgs/AEBState)
    /adas/aeb_command (geometry_msgs/AccelStamped)
    /adas/heartbeat (std_msgs/Header)
    /carla/ego_vehicle/vehicle_control_cmd (carla_msgs/CarlaEgoVehicleControl) - DIRECT CONTROL
    
Services:
    /adas/set_aeb_mode (adas_msgs/SetAEBMode)
"""

import rospy
import numpy as np
import sys
import os
from enum import IntEnum
from geometry_msgs.msg import AccelStamped
from std_msgs.msg import Header, Bool

# Add carla-ros-bridge Python path for carla_msgs
carla_bridge_path = os.path.expanduser('~/carla-ros-bridge/catkin_ws/devel/lib/python3/dist-packages')
if os.path.exists(carla_bridge_path) and carla_bridge_path not in sys.path:
    sys.path.insert(0, carla_bridge_path)

from adas_msgs.msg import TTCInfo, AEBState
from adas_msgs.srv import SetAEBMode, SetAEBModeResponse

# Import CARLA messages for direct control
try:
    from carla_msgs.msg import CarlaEgoVehicleControl
    CARLA_AVAILABLE = True
except ImportError:
    CARLA_AVAILABLE = False


class AEBStateEnum(IntEnum):
    INACTIVE = 0
    WARNING = 1
    PARTIAL_BRAKE = 2
    FULL_BRAKE = 3


class AEBFiniteStateMachine:
    def __init__(self):
        rospy.init_node('aeb_fsm', anonymous=False)
        
        # Load thresholds from parameters
        self.ttc_inactive = rospy.get_param('~ttc_inactive', 3.0)
        self.ttc_warning = rospy.get_param('~ttc_warning', 1.5)
        self.ttc_partial = rospy.get_param('~ttc_partial', 1.0)
        
        self.decel_partial_min = rospy.get_param('~decel_partial_min', 4.0)
        self.decel_partial_max = rospy.get_param('~decel_partial_max', 7.0)
        self.decel_full = rospy.get_param('~decel_full', 1.0)  # brake pedal value, not m/s²
        
        self.heartbeat_rate = rospy.get_param('~heartbeat_rate', 40.0)  # Hz (25ms period)
        
        # State stability - 降低确认要求以便调试
        self.state_confirm_count = rospy.get_param('~state_confirm_count', 1)  # 改为1帧立即响应
        
        # State variables
        self.current_state = AEBStateEnum.INACTIVE
        self.state_entry_time = rospy.Time.now()
        self.ttc_at_entry = -1.0
        self.enabled = True
        self.fault_detected = False
        self.fault_message = ""
        
        # State confirmation tracking
        self.pending_state = AEBStateEnum.INACTIVE
        self.pending_count = 0
        
        # Direct CARLA control flag
        self.direct_carla_control = rospy.get_param('~direct_carla_control', True)
        self.role_name = rospy.get_param('~role_name', 'ego_vehicle')
        
        # Publishers
        self.state_pub = rospy.Publisher('/adas/aeb_state', AEBState, queue_size=1)
        self.command_pub = rospy.Publisher('/adas/aeb_command', AccelStamped, queue_size=1)
        self.heartbeat_pub = rospy.Publisher('/adas/heartbeat', Header, queue_size=1)
        
        # Direct CARLA control publishers (bypass vehicle_controller for immediate response)
        if self.direct_carla_control and CARLA_AVAILABLE:
            self.carla_control_pub = rospy.Publisher(
                f'/carla/{self.role_name}/vehicle_control_cmd',
                CarlaEgoVehicleControl,
                queue_size=1
            )
            self.carla_override_pub = rospy.Publisher(
                f'/carla/{self.role_name}/vehicle_control_manual_override',
                Bool,
                queue_size=1,
                latch=True
            )
            rospy.loginfo("AEB FSM: Direct CARLA control ENABLED")
        else:
            self.carla_control_pub = None
            self.carla_override_pub = None
            if self.direct_carla_control:
                rospy.logwarn("AEB FSM: Direct CARLA control requested but carla_msgs not available")
        
        # Subscribers
        self.ttc_sub = rospy.Subscriber(
            '/adas/ttc_info', TTCInfo, self.ttc_callback, queue_size=1
        )
        
        # Services
        self.mode_srv = rospy.Service('/adas/set_aeb_mode', SetAEBMode, self.handle_set_mode)
        
        # Heartbeat timer
        self.heartbeat_timer = rospy.Timer(
            rospy.Duration(1.0 / self.heartbeat_rate),
            self.heartbeat_callback
        )
        
        rospy.loginfo("AEB FSM initialized")
        rospy.loginfo(f"  Thresholds - Inactive: {self.ttc_inactive}s, Warning: {self.ttc_warning}s, Partial: {self.ttc_partial}s")
        rospy.loginfo(f"  Heartbeat rate: {self.heartbeat_rate}Hz ({1000/self.heartbeat_rate:.1f}ms)")
    
    def handle_set_mode(self, req):
        """Service handler for enabling/disabling AEB"""
        self.enabled = req.enable
        status = "enabled" if self.enabled else "disabled"
        rospy.loginfo(f"AEB system {status}")
        
        if not self.enabled:
            self._transition_to(AEBStateEnum.INACTIVE)
        
        return SetAEBModeResponse(success=True, message=f"AEB {status}")
    
    def heartbeat_callback(self, event):
        """Publish heartbeat for safety monitor"""
        heartbeat = Header()
        heartbeat.stamp = rospy.Time.now()
        heartbeat.frame_id = "aeb_fsm"
        self.heartbeat_pub.publish(heartbeat)
    
    def ttc_callback(self, msg: TTCInfo):
        """Process TTC and update FSM state"""
        if not self.enabled:
            return
        
        ttc = msg.ttc
        
        # Determine target state based on TTC
        if ttc < 0 or ttc > self.ttc_inactive:
            target_state = AEBStateEnum.INACTIVE
        elif ttc >= self.ttc_warning:
            target_state = AEBStateEnum.WARNING
        elif ttc >= self.ttc_partial:
            target_state = AEBStateEnum.PARTIAL_BRAKE
        else:
            target_state = AEBStateEnum.FULL_BRAKE
        
        # State transition with confirmation (prevents single-frame false triggers)
        if target_state != self.current_state:
            # Track pending state
            if target_state == self.pending_state:
                self.pending_count += 1
            else:
                self.pending_state = target_state
                self.pending_count = 1
            
            # Require confirmation for state changes
            # Higher urgency (FULL_BRAKE) requires fewer confirmations
            required_confirms = 1 if target_state == AEBStateEnum.FULL_BRAKE else self.state_confirm_count
            
            if self.pending_count >= required_confirms:
                if target_state > self.current_state:
                    # Transition to higher urgency immediately after confirmation
                    self._transition_to(target_state, ttc)
                else:
                    # Transitions to lower urgency require minimum dwell time
                    dwell_time = (rospy.Time.now() - self.state_entry_time).to_sec()
                    
                    # CRITICAL: In FULL_BRAKE, don't release until safe or longer dwell
                    if self.current_state == AEBStateEnum.FULL_BRAKE:
                        # Require much longer dwell time to exit FULL_BRAKE
                        # This prevents premature release when obstacle detection flickers
                        min_dwell = 2.0  # 2 seconds minimum in FULL_BRAKE
                        # Also require multiple confirmations
                        if dwell_time > min_dwell and self.pending_count >= 3:
                            self._transition_to(target_state, ttc)
                    else:
                        min_dwell = 0.5  # seconds
                        if dwell_time > min_dwell:
                            self._transition_to(target_state, ttc)
        else:
            # Reset pending if back to current state
            self.pending_state = self.current_state
            self.pending_count = 0
        
        # Calculate deceleration command
        decel_cmd = self._calculate_deceleration(ttc)
        
        # Publish state and command
        self._publish_state(msg.header, decel_cmd)
        self._publish_command(msg.header, decel_cmd)
    
    def _transition_to(self, new_state: AEBStateEnum, ttc: float = -1.0):
        """Transition to new state"""
        old_state = self.current_state
        self.current_state = new_state
        self.state_entry_time = rospy.Time.now()
        self.ttc_at_entry = ttc
        
        state_names = ['INACTIVE', 'WARNING', 'PARTIAL_BRAKE', 'FULL_BRAKE']
        rospy.loginfo(f"AEB State: {state_names[old_state]} -> {state_names[new_state]} (TTC: {ttc:.2f}s)")
        
        # Direct CARLA control on state transition
        if self.carla_control_pub is not None:
            if new_state >= AEBStateEnum.PARTIAL_BRAKE:
                # Take over control from manual
                self.carla_override_pub.publish(Bool(data=False))
                rospy.logwarn(f"AEB: Taking over CARLA control (state={state_names[new_state]})")
            elif old_state >= AEBStateEnum.PARTIAL_BRAKE and new_state < AEBStateEnum.PARTIAL_BRAKE:
                # Release control back to manual
                self.carla_override_pub.publish(Bool(data=True))
                rospy.loginfo("AEB: Releasing CARLA control back to manual")
    
    def _calculate_deceleration(self, ttc: float) -> float:
        """Calculate deceleration based on current state and TTC"""
        if self.current_state == AEBStateEnum.INACTIVE:
            return 0.0
        
        elif self.current_state == AEBStateEnum.WARNING:
            return 0.0  # Warning only, no braking
        
        elif self.current_state == AEBStateEnum.PARTIAL_BRAKE:
            # Interpolate deceleration based on TTC
            # Lower TTC = higher deceleration
            ttc_range = self.ttc_warning - self.ttc_partial
            ttc_normalized = (self.ttc_warning - max(ttc, self.ttc_partial)) / ttc_range
            decel = self.decel_partial_min + ttc_normalized * (self.decel_partial_max - self.decel_partial_min)
            return decel
        
        elif self.current_state == AEBStateEnum.FULL_BRAKE:
            return self.decel_full
        
        return 0.0
    
    def _publish_state(self, header, decel_cmd: float):
        """Publish AEB state message"""
        msg = AEBState()
        msg.header = header
        msg.state = int(self.current_state)
        msg.deceleration_cmd = decel_cmd
        msg.state_duration = (rospy.Time.now() - self.state_entry_time).to_sec()
        msg.ttc_at_entry = self.ttc_at_entry
        msg.enabled = self.enabled
        msg.fault_detected = self.fault_detected
        msg.fault_message = self.fault_message
        
        self.state_pub.publish(msg)
    
    def _publish_command(self, header, decel_cmd: float):
        """Publish deceleration command and direct CARLA control"""
        msg = AccelStamped()
        msg.header = header
        msg.accel.linear.x = -decel_cmd  # Negative = deceleration
        msg.accel.linear.y = 0.0
        msg.accel.linear.z = 0.0
        
        self.command_pub.publish(msg)
        
        # Direct CARLA control - bypass vehicle_controller for immediate braking
        if self.carla_control_pub is not None and self.current_state >= AEBStateEnum.PARTIAL_BRAKE:
            carla_cmd = CarlaEgoVehicleControl()
            carla_cmd.header.stamp = rospy.Time.now()
            
            if self.current_state == AEBStateEnum.FULL_BRAKE:
                carla_cmd.brake = 1.0
                carla_cmd.throttle = 0.0
            elif self.current_state == AEBStateEnum.PARTIAL_BRAKE:
                carla_cmd.brake = 0.6
                carla_cmd.throttle = 0.0
            
            carla_cmd.steer = 0.0
            carla_cmd.hand_brake = False
            carla_cmd.reverse = False
            carla_cmd.manual_gear_shift = False
            
            self.carla_control_pub.publish(carla_cmd)
            
            # Also keep publishing override=False
            self.carla_override_pub.publish(Bool(data=False))
            
            # Log for debugging
            if not hasattr(self, '_brake_log_counter'):
                self._brake_log_counter = 0
            self._brake_log_counter += 1
            if self._brake_log_counter % 20 == 1:
                rospy.logwarn(f"AEB DIRECT BRAKE: state={self.current_state}, brake={carla_cmd.brake}")
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = AEBFiniteStateMachine()
        node.run()
    except rospy.ROSInterruptException:
        pass
