#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Time-to-Collision (TTC) Calculator
碰撞时间计算器 - 使用二阶TTC模型

Model: S + V_rel * TTC + 0.5 * β * TTC² = 0

Subscribed Topics:
    /adas/tracked_obstacles (adas_msgs/ObstacleArray)
    /adas/ego_state (geometry_msgs/TwistStamped)
    
Published Topics:
    /adas/ttc_info (adas_msgs/TTCInfo)
"""

import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped

from adas_msgs.msg import ObstacleArray, TTCInfo


class TTCCalculator:
    def __init__(self):
        rospy.init_node('ttc_calculator', anonymous=False)
        
        # Parameters
        self.min_ttc = rospy.get_param('~min_ttc', 0.1)  # seconds
        self.max_ttc = rospy.get_param('~max_ttc', 10.0)  # seconds
        self.safety_margin = rospy.get_param('~safety_margin', 2.0)  # meters
        
        # Stability filtering parameters - 放宽条件以便调试
        self.min_ego_velocity = rospy.get_param('~min_ego_velocity', 0.1)  # m/s - 降低到0.1
        self.min_obstacle_confidence = rospy.get_param('~min_obstacle_confidence', 0.1)  # 降低
        self.stable_frames_required = rospy.get_param('~stable_frames_required', 1)  # 改为1帧立即响应
        
        # State
        self.ego_velocity = 0.0  # m/s (longitudinal)
        self.ego_acceleration = 0.0  # m/s^2
        self.last_velocity_time = None
        
        # Obstacle stability tracking
        self.obstacle_frame_count = 0
        self.last_closest_id = -1
        
        # Publishers
        self.ttc_pub = rospy.Publisher('/adas/ttc_info', TTCInfo, queue_size=1)
        
        # Subscribers
        self.obstacle_sub = rospy.Subscriber(
            '/adas/tracked_obstacles', ObstacleArray, self.obstacle_callback, queue_size=1
        )
        self.ego_sub = rospy.Subscriber(
            '/adas/ego_state', TwistStamped, self.ego_callback, queue_size=1
        )
        
        rospy.loginfo("TTC Calculator initialized")
    
    def ego_callback(self, msg: TwistStamped):
        """Update ego vehicle state"""
        new_velocity = msg.twist.linear.x  # Forward velocity
        current_time = msg.header.stamp
        
        # Estimate acceleration
        if self.last_velocity_time is not None:
            dt = (current_time - self.last_velocity_time).to_sec()
            if dt > 0.01:
                self.ego_acceleration = (new_velocity - self.ego_velocity) / dt
        
        self.ego_velocity = new_velocity
        self.last_velocity_time = current_time
    
    def obstacle_callback(self, msg: ObstacleArray):
        """Calculate TTC for closest obstacle"""
        ttc_msg = TTCInfo()
        ttc_msg.header = msg.header
        ttc_msg.ttc = -1.0  # Default: no collision risk
        ttc_msg.collision_imminent = False
        
        # Debug: log ego velocity periodically
        if not hasattr(self, '_log_counter'):
            self._log_counter = 0
        self._log_counter += 1
        
        # Skip TTC calculation if ego velocity too low (stationary/crawling)
        if abs(self.ego_velocity) < self.min_ego_velocity:
            if self._log_counter % 100 == 0:  # Log every 5 seconds at 20Hz
                rospy.loginfo(f"TTC skip: ego_velocity={self.ego_velocity:.2f} < {self.min_ego_velocity}")
            self.obstacle_frame_count = 0  # Reset stability counter
            self.ttc_pub.publish(ttc_msg)
            return
        
        if msg.closest_obstacle_idx < 0 or len(msg.obstacles) == 0:
            if self._log_counter % 100 == 0:
                rospy.loginfo(f"TTC skip: no obstacles (count={len(msg.obstacles)}, closest_idx={msg.closest_obstacle_idx})")
            self.obstacle_frame_count = 0
            self.ttc_pub.publish(ttc_msg)
            return
        
        # Get closest obstacle
        closest = msg.obstacles[msg.closest_obstacle_idx]
        
        # 移除置信度过滤 - LiDAR处理器已经过滤了噪声
        # confidence现在总是 >= 0.3（基于点数计算）
        
        # Track obstacle stability (same obstacle for multiple frames)
        if closest.id == self.last_closest_id:
            self.obstacle_frame_count += 1
        else:
            self.obstacle_frame_count = 1
            self.last_closest_id = closest.id
        
        # Require stable detection before computing TTC
        if self.obstacle_frame_count < self.stable_frames_required:
            self.ttc_pub.publish(ttc_msg)
            return
        
        # Calculate relative velocity (positive = approaching)
        # 关键修复：使用ego速度来计算相对速度
        # 如果障碍物是静止的（地面噪声），它的velocity接近0
        # 相对速度 = ego速度 - 障碍物速度（沿车辆前进方向）
        obstacle_vel_x = closest.velocity.x if hasattr(closest.velocity, 'x') else 0.0
        
        # v_rel > 0 表示接近，v_rel < 0 表示远离
        # ego前进时，静止障碍物的相对速度 = ego_velocity
        v_rel = self.ego_velocity - obstacle_vel_x
        
        # 额外检查：如果距离很近但相对速度很小，可能是噪声
        # 真正的碰撞风险需要足够的相对速度
        min_approach_velocity = 0.5  # m/s - 至少0.5m/s的接近速度才算风险
        if v_rel < min_approach_velocity:
            if self._log_counter % 100 == 0:
                rospy.loginfo(f"TTC skip: low approach velocity v_rel={v_rel:.2f} < {min_approach_velocity}")
            self.ttc_pub.publish(ttc_msg)
            return
        
        # Relative acceleration (assume obstacle maintains constant velocity)
        a_rel = -self.ego_acceleration
        
        # Distance with safety margin
        distance = closest.distance - self.safety_margin
        
        # 额外检查：距离过近可能是车身反射或地面噪声
        min_valid_distance = 1.0  # 至少1米才是有效障碍物
        if closest.distance < min_valid_distance:
            if self._log_counter % 100 == 0:
                rospy.loginfo(f"TTC skip: too close distance={closest.distance:.2f} < {min_valid_distance} (possible sensor noise)")
            self.ttc_pub.publish(ttc_msg)
            return
        
        # 调试日志
        if self._log_counter % 20 == 0:  # 每秒一次
            rospy.loginfo(f"TTC calc: dist={closest.distance:.1f}m, v_rel={v_rel:.2f}m/s, ego_v={self.ego_velocity:.2f}m/s")
        
        if distance <= 0:
            # Already within safety margin
            ttc_msg.ttc = 0.0
            ttc_msg.collision_imminent = True
        else:
            # Calculate TTC using second-order model
            ttc = self._calculate_ttc_second_order(distance, v_rel, a_rel)
            ttc_msg.ttc = ttc
            ttc_msg.collision_imminent = (0 < ttc < 0.6)
        
        # Fill message
        ttc_msg.distance = closest.distance
        ttc_msg.relative_velocity = v_rel
        ttc_msg.relative_acceleration = a_rel
        ttc_msg.confidence = closest.confidence
        
        self.ttc_pub.publish(ttc_msg)
        
        # Debug logging
        if ttc_msg.ttc > 0 and ttc_msg.ttc < self.max_ttc:
            rospy.logdebug(f"TTC: {ttc_msg.ttc:.2f}s, Distance: {distance:.1f}m, V_rel: {v_rel:.1f}m/s")
    
    def _calculate_ttc_second_order(self, s: float, v_rel: float, a_rel: float) -> float:
        """
        Calculate TTC using second-order kinematic model
        
        Solve: s + v_rel * t + 0.5 * a_rel * t² = 0
        
        Args:
            s: Distance to obstacle (meters)
            v_rel: Relative velocity (m/s), positive = approaching
            a_rel: Relative acceleration (m/s²)
            
        Returns:
            TTC in seconds, or -1 if no collision predicted
        """
        # Case 1: Vehicles separating
        if v_rel <= 0 and a_rel <= 0:
            return -1.0
        
        # Case 2: Constant velocity (a_rel ≈ 0)
        if abs(a_rel) < 0.01:
            if v_rel > 0:
                ttc = s / v_rel
                return min(max(ttc, self.min_ttc), self.max_ttc)
            return -1.0
        
        # Case 3: Second-order equation
        # 0.5 * a * t² + v * t + s = 0
        # Using quadratic formula
        a = 0.5 * a_rel
        b = v_rel
        c = -s  # Note: we want s + vt + 0.5at² = 0, rearranged
        
        discriminant = b**2 - 4*a*c
        
        if discriminant < 0:
            # No real solution - no collision
            return -1.0
        
        sqrt_disc = np.sqrt(discriminant)
        t1 = (-b + sqrt_disc) / (2*a)
        t2 = (-b - sqrt_disc) / (2*a)
        
        # Find smallest positive solution
        solutions = [t for t in [t1, t2] if t > self.min_ttc]
        
        if not solutions:
            return -1.0
        
        ttc = min(solutions)
        return min(ttc, self.max_ttc)
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = TTCCalculator()
        node.run()
    except rospy.ROSInterruptException:
        pass
