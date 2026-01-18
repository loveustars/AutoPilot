#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Time-to-Collision (TTC) Calculator
碰撞时间计算器 - 使用二阶TTC模型

改进:
  - 行驶走廊检测 (只检测路径上的障碍物)
  - 转弯动态走廊 (根据yaw_rate调整)
  - 鬼探头检测 (侧向入侵)
  - 标志牌过滤 (排除悬空物体)

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
        self.safety_margin = rospy.get_param('~safety_margin', 2.0)  # meters (longitudinal)
        
        # 行驶走廊参数
        self.ego_width = rospy.get_param('~ego_width', 2.0)  # 车宽 (meters)
        self.lane_margin = rospy.get_param('~lane_margin', 1.0)  # 车道边缘余量 (meters) - 放宽
        self.corridor_half_width = (self.ego_width / 2.0) + self.lane_margin  # 基础走廊半宽 = 2.0m
        
        # 转弯处理参数
        self.turning_enabled = rospy.get_param('~turning_enabled', True)
        self.yaw_rate_threshold = rospy.get_param('~yaw_rate_threshold', 0.05)  # rad/s - 超过此值认为在转弯
        self.turning_corridor_expansion = rospy.get_param('~turning_corridor_expansion', 0.5)  # 转弯时走廊扩展 (meters)
        self.curve_lookahead = rospy.get_param('~curve_lookahead', 2.0)  # 弯道前瞻时间 (seconds)
        
        # 鬼探头检测参数
        self.lateral_intrusion_enabled = rospy.get_param('~lateral_intrusion_enabled', True)
        self.lateral_detection_width = rospy.get_param('~lateral_detection_width', 4.0)
        self.lateral_velocity_threshold = rospy.get_param('~lateral_velocity_threshold', 0.8)  # 提高阈值
        self.intrusion_ttc_threshold = rospy.get_param('~intrusion_ttc_threshold', 1.5)
        
        # 标志牌过滤参数 (排除悬空的标志牌)
        self.sign_filter_enabled = rospy.get_param('~sign_filter_enabled', True)
        self.sign_width_threshold = rospy.get_param('~sign_width_threshold', 0.8)  # 标志牌最大宽度
        self.sign_height_ratio = rospy.get_param('~sign_height_ratio', 2.0)  # 高度/宽度比 > 此值认为是杆
        self.sign_min_ground_clearance = rospy.get_param('~sign_min_ground_clearance', 1.0)  # 底部离地高度
        
        # Stability filtering
        self.min_ego_velocity = rospy.get_param('~min_ego_velocity', 0.3)  # m/s - 降低阈值
        self.stable_frames_required = rospy.get_param('~stable_frames_required', 1)  # 只需要1帧即可
        
        # State
        self.ego_velocity = 0.0  # m/s (longitudinal)
        self.ego_acceleration = 0.0  # m/s^2
        self.ego_yaw_rate = 0.0  # rad/s (angular.z)
        self.last_velocity_time = None
        
        # Obstacle stability tracking
        self.obstacle_history = {}  # {id: frame_count}
        
        # Publishers
        self.ttc_pub = rospy.Publisher('/adas/ttc_info', TTCInfo, queue_size=1)
        
        # Subscribers - 直接订阅obstacles而非tracked_obstacles
        self.obstacle_sub = rospy.Subscriber(
            '/adas/obstacles', ObstacleArray, self.obstacle_callback, queue_size=1
        )
        self.ego_sub = rospy.Subscriber(
            '/adas/ego_state', TwistStamped, self.ego_callback, queue_size=1
        )
        
        rospy.loginfo("TTC Calculator initialized")
        rospy.loginfo(f"  Base corridor half-width: {self.corridor_half_width}m")
        rospy.loginfo(f"  Turning handling: {'enabled' if self.turning_enabled else 'disabled'}")
        rospy.loginfo(f"  Sign filtering: {'enabled' if self.sign_filter_enabled else 'disabled'}")
    
    def ego_callback(self, msg: TwistStamped):
        """Update ego vehicle state including yaw rate for turning detection"""
        new_velocity = msg.twist.linear.x  # Forward velocity
        current_time = msg.header.stamp
        
        # Get yaw rate for turning detection
        self.ego_yaw_rate = msg.twist.angular.z  # rad/s
        
        # Estimate acceleration
        if self.last_velocity_time is not None:
            dt = (current_time - self.last_velocity_time).to_sec()
            if dt > 0.01:
                self.ego_acceleration = (new_velocity - self.ego_velocity) / dt
        
        self.ego_velocity = new_velocity
        self.last_velocity_time = current_time
    
    def _is_sign_or_pole(self, obstacle) -> bool:
        """
        检测障碍物是否为悬空标志牌（应该忽略）
        
        只过滤悬空物体（如标志牌伸入车道上方）
        地面上的任何物体都不过滤（包括行人、车辆、路障等）
        
        返回True表示应该忽略此障碍物
        """
        if not self.sign_filter_enabled:
            return False
        
        height = obstacle.height
        obs_z = obstacle.position.z  # 障碍物中心高度
        
        # 估计底部离地高度 (中心z - 高度/2)
        # 在CARLA中，z=0是LiDAR高度，地面约在z=-2.4
        ground_clearance = (obs_z - height / 2.0) + 2.4  # 相对于地面的高度
        
        # 唯一过滤条件：悬空物体
        # 如果底部离地超过阈值（如1m），说明是悬空的标志牌，不是真正的障碍物
        # 地面上的任何物体（ground_clearance <= 1.0m）都不过滤
        if ground_clearance > self.sign_min_ground_clearance:
            if self._log_counter % 50 == 0:
                rospy.logdebug(f"过滤悬空物体: z={obs_z:.1f}, h={height:.1f}, 离地={ground_clearance:.1f}m")
            return True
        
        return False
    
    def _get_dynamic_corridor(self, obs_x: float) -> float:
        """
        根据车辆转弯状态和障碍物距离计算动态走廊宽度
        
        转弯时，远处的障碍物可能在当前直线走廊外，但在预测路径上
        需要根据yaw_rate预测车辆将要到达的位置
        """
        corridor = self.corridor_half_width
        
        if not self.turning_enabled:
            return corridor
        
        # 如果在转弯中
        if abs(self.ego_yaw_rate) > self.yaw_rate_threshold:
            # 计算到达障碍物时的转向偏移量
            # 预计到达时间
            if self.ego_velocity > 0.5:
                time_to_obstacle = obs_x / self.ego_velocity
                # 转弯期间的横向偏移（简化模型）
                # lateral_offset ≈ v * yaw_rate * t
                predicted_lateral_offset = abs(self.ego_velocity * self.ego_yaw_rate * time_to_obstacle)
                # 扩展走廊以包含预测路径
                corridor += min(predicted_lateral_offset, self.turning_corridor_expansion * 2)
        
        return corridor
    
    def obstacle_callback(self, msg: ObstacleArray):
        """Calculate TTC for obstacles in driving corridor with turning and sign filtering"""
        ttc_msg = TTCInfo()
        ttc_msg.header = msg.header
        ttc_msg.ttc = -1.0
        ttc_msg.collision_imminent = False
        
        # Debug counter
        if not hasattr(self, '_log_counter'):
            self._log_counter = 0
        self._log_counter += 1
        
        # Skip if ego velocity too low
        if abs(self.ego_velocity) < self.min_ego_velocity:
            if self._log_counter % 50 == 0:
                rospy.loginfo(f"TTC跳过: 速度过低 {self.ego_velocity:.2f} < {self.min_ego_velocity}")
            self.ttc_pub.publish(ttc_msg)
            return
        
        if len(msg.obstacles) == 0:
            if self._log_counter % 50 == 0:
                rospy.loginfo("TTC跳过: 无障碍物")
            self.ttc_pub.publish(ttc_msg)
            return
        
        # 打印障碍物信息
        if self._log_counter % 20 == 0:
            rospy.loginfo(f"收到{len(msg.obstacles)}个障碍物, ego_v={self.ego_velocity:.1f}m/s, corridor={self.corridor_half_width:.1f}m")
        
        # 更新障碍物稳定性计数
        current_ids = set()
        for obs in msg.obstacles:
            current_ids.add(obs.id)
            if obs.id not in self.obstacle_history:
                self.obstacle_history[obs.id] = 1
            else:
                self.obstacle_history[obs.id] += 1
        
        # 清理消失的障碍物
        disappeared = [oid for oid in self.obstacle_history if oid not in current_ids]
        for oid in disappeared:
            del self.obstacle_history[oid]
        
        # 检测是否在转弯
        is_turning = abs(self.ego_yaw_rate) > self.yaw_rate_threshold
        if is_turning and self._log_counter % 40 == 0:
            direction = "左" if self.ego_yaw_rate > 0 else "右"
            rospy.loginfo(f"转弯中: {direction}转, yaw_rate={self.ego_yaw_rate:.3f}rad/s")
        
        # 找到最危险的障碍物
        best_obstacle = None
        best_ttc = float('inf')
        best_reason = ""
        
        for obstacle in msg.obstacles:
            # 检查稳定性 - 需要连续检测到
            if self.obstacle_history.get(obstacle.id, 0) < self.stable_frames_required:
                continue
            
            # 获取障碍物信息
            obs_x = obstacle.position.x
            obs_y = obstacle.position.y
            obs_vx = obstacle.velocity.x if hasattr(obstacle.velocity, 'x') else 0.0
            obs_vy = obstacle.velocity.y if hasattr(obstacle.velocity, 'y') else 0.0
            
            # 过滤标志牌/路灯柱
            if self._is_sign_or_pole(obstacle):
                if self._log_counter % 100 == 0:
                    rospy.logdebug(f"过滤标志牌: id={obstacle.id}, pos=({obs_x:.1f}, {obs_y:.1f})")
                continue
            
            # 获取动态走廊宽度（考虑转弯）
            dynamic_corridor = self._get_dynamic_corridor(obs_x)
            
            # 转弯时，需要考虑车辆将要到达的位置
            # 预测障碍物相对于未来车辆位置的横向偏移
            if is_turning and self.ego_velocity > 0.5:
                time_to_obstacle = obs_x / self.ego_velocity
                # 车辆转弯后的横向偏移
                # 如果左转(yaw_rate>0)，车辆会向左移动，相当于障碍物向右移动
                predicted_obs_y = obs_y - self.ego_velocity * self.ego_yaw_rate * time_to_obstacle * 0.5
            else:
                predicted_obs_y = obs_y
            
            # 检查是否在（动态）走廊内
            in_corridor = abs(predicted_obs_y) <= dynamic_corridor
            
            # 鬼探头检测 - 只在非转弯时或转弯角度小时启用
            is_intruding = False
            intrusion_ttc = float('inf')
            
            if self.lateral_intrusion_enabled and not in_corridor:
                # 转弯时提高鬼探头检测阈值，减少误报
                effective_lateral_threshold = self.lateral_velocity_threshold
                if is_turning:
                    effective_lateral_threshold *= 1.5  # 转弯时需要更高的横向速度才算鬼探头
                
                if abs(obs_y) <= self.lateral_detection_width:
                    lateral_approach_velocity = -np.sign(obs_y) * obs_vy
                    
                    if lateral_approach_velocity > effective_lateral_threshold:
                        distance_to_corridor = abs(obs_y) - dynamic_corridor
                        intrusion_ttc = distance_to_corridor / lateral_approach_velocity
                        
                        if intrusion_ttc < self.intrusion_ttc_threshold:
                            is_intruding = True
                            if self._log_counter % 30 == 0:
                                side = "左" if obs_y > 0 else "右"
                                rospy.logwarn(f"鬼探头: {side}侧, vy={obs_vy:.1f}m/s, {intrusion_ttc:.1f}s后进入")
            
            # 只处理在走廊内或正在侵入的障碍物
            if not in_corridor and not is_intruding:
                continue
            
            # 计算TTC
            v_rel = self.ego_velocity - obs_vx
            if v_rel < 0.3:
                continue
            
            distance = obstacle.distance - self.safety_margin
            if distance <= 0:
                ttc = 0.0
            else:
                ttc = self._calculate_ttc_second_order(distance, v_rel, -self.ego_acceleration)
            
            if is_intruding:
                effective_ttc = min(ttc if ttc > 0 else float('inf'), intrusion_ttc)
                reason = "lateral_intrusion"
            else:
                effective_ttc = ttc
                reason = "in_corridor"
            
            if 0 < effective_ttc < best_ttc:
                best_ttc = effective_ttc
                best_obstacle = obstacle
                best_reason = reason
        
        # 填充消息
        if best_obstacle is not None and best_ttc < self.max_ttc:
            ttc_msg.ttc = best_ttc
            ttc_msg.collision_imminent = (0 < best_ttc < 0.6)
            ttc_msg.distance = best_obstacle.distance
            ttc_msg.relative_velocity = self.ego_velocity - (best_obstacle.velocity.x if hasattr(best_obstacle.velocity, 'x') else 0.0)
            ttc_msg.relative_acceleration = -self.ego_acceleration
            ttc_msg.confidence = best_obstacle.confidence
            
            if self._log_counter % 20 == 0:
                rospy.loginfo(f"TTC: {best_ttc:.2f}s, dist={best_obstacle.distance:.1f}m, y={best_obstacle.position.y:.1f}m, reason={best_reason}")
        
        self.ttc_pub.publish(ttc_msg)
    
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
