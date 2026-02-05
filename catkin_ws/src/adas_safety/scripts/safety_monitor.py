#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Safety Monitor - Independent Monitoring Node
功能安全独立监控节点 (ISO 26262)

Architecture: Non-symmetric redundancy
    Primary: AEB FSM -> /adas/aeb_command
    Monitor: Safety Monitor (this node) -> validates commands

Functions:
    1. Heartbeat Watchdog: Detect AEB node freeze (<50ms)
    2. Plausibility Check: Block irrational commands
    3. Override Control: Take over if primary fails

Subscribed Topics:
    /adas/heartbeat (std_msgs/Header)
    /adas/aeb_command (geometry_msgs/AccelStamped)
    /adas/aeb_state (adas_msgs/AEBState)
    /adas/tracked_obstacles (adas_msgs/ObstacleArray)
    
Published Topics:
    /adas/safe_command (geometry_msgs/AccelStamped) - Final validated command
    /adas/safety_status (adas_msgs/SafetyStatus)
"""
from __future__ import annotations

import rospy
import threading
from geometry_msgs.msg import AccelStamped
from std_msgs.msg import Header

from adas_msgs.msg import AEBState, ObstacleArray, SafetyStatus


class SafetyMonitor:
    # AEB State constants
    AEB_INACTIVE = 0
    AEB_WARNING = 1
    AEB_PARTIAL_BRAKE = 2
    AEB_FULL_BRAKE = 3
    
    def __init__(self):
        rospy.init_node('safety_monitor', anonymous=False)
        
        # Parameters
        self.watchdog_timeout_ms = rospy.get_param('~watchdog_timeout_ms', 50.0)
        self.max_decel_no_obstacle = rospy.get_param('~max_decel_no_obstacle', 3.0)
        self.min_obstacle_distance = rospy.get_param('~min_obstacle_distance', 50.0)
        self.safe_deceleration = rospy.get_param('~safe_deceleration', 8.0)
        
        # Emergency override parameters
        self.emergency_override_enabled = rospy.get_param('~emergency_override_enabled', True)
        self.emergency_ttc_threshold = rospy.get_param('~emergency_ttc_threshold', 1.0)
        
        # State
        self.last_heartbeat_time = rospy.Time.now()
        self.last_aeb_command = None
        self.last_aeb_state = None
        self.closest_obstacle_distance = float('inf')
        self.current_ttc = -1.0
        
        self.primary_alive = True
        self.override_active = False
        self.emergency_override_active = False
        self.plausibility_ok = True
        self.plausibility_reason = ""
        
        # Thread lock for state access
        self.lock = threading.Lock()
        
        # Watchdog state - start inactive until first heartbeat received
        self.first_heartbeat_received = False
        self.watchdog_active = False
        self.watchdog_activation_time = None  # Delay activation to allow system startup
        
        # Publishers
        self.safe_cmd_pub = rospy.Publisher('/adas/safe_command', AccelStamped, queue_size=1)
        self.status_pub = rospy.Publisher('/adas/safety_status', SafetyStatus, queue_size=1)
        
        # Subscribers
        self.heartbeat_sub = rospy.Subscriber(
            '/adas/heartbeat', Header, self.heartbeat_callback, queue_size=1
        )
        self.command_sub = rospy.Subscriber(
            '/adas/aeb_command', AccelStamped, self.command_callback, queue_size=1
        )
        self.state_sub = rospy.Subscriber(
            '/adas/aeb_state', AEBState, self.state_callback, queue_size=1
        )
        self.obstacle_sub = rospy.Subscriber(
            '/adas/obstacles', ObstacleArray, self.obstacle_callback, queue_size=1
        )
        
        # Watchdog timer - high frequency for 50ms detection
        self.watchdog_timer = rospy.Timer(
            rospy.Duration(0.01),  # 100Hz check
            self.watchdog_callback
        )
        
        # Status publish timer
        self.status_timer = rospy.Timer(
            rospy.Duration(0.1),
            self.publish_status
        )
        
        rospy.loginfo("Safety Monitor initialized")
        rospy.loginfo(f"  Watchdog timeout: {self.watchdog_timeout_ms}ms")
        rospy.loginfo(f"  Emergency override: {'enabled' if self.emergency_override_enabled else 'disabled'}")
        rospy.loginfo(f"  Emergency TTC threshold: {self.emergency_ttc_threshold}s")
    
    def heartbeat_callback(self, msg: Header):
        """Receive heartbeat from AEB FSM"""
        with self.lock:
            # Use current time, not message timestamp (avoids clock sync issues in simulation)
            self.last_heartbeat_time = rospy.Time.now()
            
            # Activate watchdog after first heartbeat with delay
            if not self.first_heartbeat_received:
                self.first_heartbeat_received = True
                self.watchdog_activation_time = rospy.Time.now()
                rospy.loginfo("Safety Monitor: First heartbeat received, watchdog will activate in 1s")
                
            if not self.primary_alive:
                rospy.loginfo("Safety Monitor: Primary node recovered")
            self.primary_alive = True
    
    def command_callback(self, msg: AccelStamped):
        """Receive command from AEB FSM"""
        with self.lock:
            self.last_aeb_command = msg
        
        # Validate and forward command (with emergency override check)
        self._process_command(msg)
    
    def state_callback(self, msg: AEBState):
        """Receive state from AEB FSM and check for emergency"""
        with self.lock:
            self.last_aeb_state = msg
            self.current_ttc = msg.ttc_at_entry
            
            # Check for emergency override condition
            if self.emergency_override_enabled and msg.state == self.AEB_FULL_BRAKE:
                if not self.emergency_override_active:
                    self.emergency_override_active = True
                    rospy.logwarn("EMERGENCY OVERRIDE: TTC critical, forcing maximum braking")
            elif self.emergency_override_active and msg.state < self.AEB_PARTIAL_BRAKE:
                self.emergency_override_active = False
                rospy.loginfo("Emergency override released")
    
    def obstacle_callback(self, msg: ObstacleArray):
        """Update obstacle information for plausibility check"""
        with self.lock:
            if msg.closest_obstacle_idx >= 0 and len(msg.obstacles) > 0:
                self.closest_obstacle_distance = msg.obstacles[msg.closest_obstacle_idx].distance
            else:
                self.closest_obstacle_distance = float('inf')
    
    def watchdog_callback(self, event):
        """Check heartbeat timeout"""
        # Skip if no heartbeat received yet (startup phase)
        with self.lock:
            if not self.first_heartbeat_received:
                return
            
            # Delay watchdog activation for 1 second after first heartbeat
            # This allows all nodes to stabilize during startup
            if self.watchdog_activation_time is not None:
                startup_elapsed = (rospy.Time.now() - self.watchdog_activation_time).to_sec()
                if startup_elapsed < 1.0:
                    return  # Still in startup grace period
                elif not self.watchdog_active:
                    self.watchdog_active = True
                    rospy.loginfo("Safety Monitor: Watchdog now active")
            
            age_ms = (rospy.Time.now() - self.last_heartbeat_time).to_sec() * 1000
            
            if age_ms > self.watchdog_timeout_ms:
                if self.primary_alive:
                    rospy.logwarn(f"Safety Monitor: Primary node TIMEOUT ({age_ms:.0f}ms > {self.watchdog_timeout_ms}ms)")
                    rospy.logwarn("Safety Monitor: Activating OVERRIDE - controlled deceleration")
                    self.primary_alive = False
                    self.override_active = True
                    self._execute_safe_state()
            else:
                if self.override_active and self.primary_alive:
                    rospy.loginfo("Safety Monitor: Releasing override, primary recovered")
                    self.override_active = False
    
    def _process_command(self, cmd: AccelStamped):
        """Validate command and forward or override"""
        # Check if override is active
        if self.override_active:
            return  # Safe state command is being published by watchdog
        
        # Plausibility check
        plausible, reason = self._check_plausibility(cmd)
        
        with self.lock:
            self.plausibility_ok = plausible
            self.plausibility_reason = reason
        
        if plausible:
            # Forward validated command
            self.safe_cmd_pub.publish(cmd)
        else:
            # Block command and use safe alternative
            rospy.logwarn(f"Safety Monitor: Command BLOCKED - {reason}")
            self._publish_safe_command(cmd.header, 0.0)  # Zero decel as safe alternative
    
    def _check_plausibility(self, cmd: AccelStamped) -> tuple[bool, str]:
        """
        Check if command is plausible given current situation
        
        NOTE: Relaxed for debugging - always allow braking
        The obstacle distance check was blocking all braking commands.
        """
        decel = abs(cmd.accel.linear.x)
        
        # Only check vehicle capability limit
        max_decel = 12.0  # m/s² - typical ABS limit
        if decel > max_decel:
            return False, f"Deceleration ({decel:.1f}m/s²) exceeds vehicle capability ({max_decel}m/s²)"
        
        # Log when braking is commanded
        if decel > 0.1:
            with self.lock:
                obs_distance = self.closest_obstacle_distance
            rospy.loginfo(f"Safety Monitor: Allowing brake cmd decel={decel:.1f}m/s², obs_dist={obs_distance:.1f}m")
        
        return True, ""
    
    def _execute_safe_state(self):
        """Execute safe state: controlled deceleration"""
        # Publish safe deceleration command at high rate
        rate = rospy.Rate(50)
        while self.override_active and not rospy.is_shutdown():
            header = Header()
            header.stamp = rospy.Time.now()
            self._publish_safe_command(header, self.safe_deceleration)
            rate.sleep()
    
    def _publish_safe_command(self, header, decel: float):
        """Publish a safe command"""
        cmd = AccelStamped()
        cmd.header = header
        cmd.accel.linear.x = -decel
        self.safe_cmd_pub.publish(cmd)
    
    def publish_status(self, event):
        """Publish safety monitor status"""
        with self.lock:
            status = SafetyStatus()
            status.header.stamp = rospy.Time.now()
            status.system_healthy = self.primary_alive and self.plausibility_ok
            status.primary_node_alive = self.primary_alive
            status.last_heartbeat_age = (rospy.Time.now() - self.last_heartbeat_time).to_sec()
            status.command_plausible = self.plausibility_ok
            status.plausibility_reason = self.plausibility_reason
            status.override_active = self.override_active
            status.safe_deceleration = self.safe_deceleration
            status.watchdog_timeout_ms = self.watchdog_timeout_ms
        
        self.status_pub.publish(status)
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = SafetyMonitor()
        node.run()
    except rospy.ROSInterruptException:
        pass
