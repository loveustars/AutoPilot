#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AEB Finite State Machine
自动紧急制动状态机

States:
    INACTIVE:      TTC > 2.5s
    WARNING:       1.5s <= TTC < 2.5s (FCW alert)
    PARTIAL_BRAKE: 0.6s <= TTC < 1.5s (decel 3.0-6.0 m/s²)
    FULL_BRAKE:    TTC < 0.6s (decel 10.0 m/s²)

Subscribed Topics:
    /adas/ttc_info (adas_msgs/TTCInfo)
    
Published Topics:
    /adas/aeb_state (adas_msgs/AEBState)
    /adas/aeb_command (geometry_msgs/AccelStamped)
    /adas/heartbeat (std_msgs/Header)
    
Services:
    /adas/set_aeb_mode (adas_msgs/SetAEBMode)
"""

import rospy
import numpy as np
from enum import IntEnum
from geometry_msgs.msg import AccelStamped
from std_msgs.msg import Header

from adas_msgs.msg import TTCInfo, AEBState
from adas_msgs.srv import SetAEBMode, SetAEBModeResponse


class AEBStateEnum(IntEnum):
    INACTIVE = 0
    WARNING = 1
    PARTIAL_BRAKE = 2
    FULL_BRAKE = 3


class AEBFiniteStateMachine:
    def __init__(self):
        rospy.init_node('aeb_fsm', anonymous=False)
        
        # Load thresholds from parameters
        self.ttc_inactive = rospy.get_param('~ttc_inactive', 2.5)
        self.ttc_warning = rospy.get_param('~ttc_warning', 1.5)
        self.ttc_partial = rospy.get_param('~ttc_partial', 0.6)
        
        self.decel_partial_min = rospy.get_param('~decel_partial_min', 3.0)
        self.decel_partial_max = rospy.get_param('~decel_partial_max', 6.0)
        self.decel_full = rospy.get_param('~decel_full', 10.0)
        
        self.heartbeat_rate = rospy.get_param('~heartbeat_rate', 50.0)  # Hz
        
        # State variables
        self.current_state = AEBStateEnum.INACTIVE
        self.state_entry_time = rospy.Time.now()
        self.ttc_at_entry = -1.0
        self.enabled = True
        self.fault_detected = False
        self.fault_message = ""
        
        # Publishers
        self.state_pub = rospy.Publisher('/adas/aeb_state', AEBState, queue_size=1)
        self.command_pub = rospy.Publisher('/adas/aeb_command', AccelStamped, queue_size=1)
        self.heartbeat_pub = rospy.Publisher('/adas/heartbeat', Header, queue_size=1)
        
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
        
        # State transition (with hysteresis to prevent oscillation)
        if target_state != self.current_state:
            # Only allow transitions to higher urgency states immediately
            # Transitions to lower urgency require minimum dwell time
            if target_state > self.current_state:
                self._transition_to(target_state, ttc)
            else:
                # Check minimum dwell time (prevents oscillation)
                dwell_time = (rospy.Time.now() - self.state_entry_time).to_sec()
                min_dwell = 0.3  # seconds
                if dwell_time > min_dwell:
                    self._transition_to(target_state, ttc)
        
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
        """Publish deceleration command"""
        msg = AccelStamped()
        msg.header = header
        msg.accel.linear.x = -decel_cmd  # Negative = deceleration
        msg.accel.linear.y = 0.0
        msg.accel.linear.z = 0.0
        
        self.command_pub.publish(msg)
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = AEBFiniteStateMachine()
        node.run()
    except rospy.ROSInterruptException:
        pass
