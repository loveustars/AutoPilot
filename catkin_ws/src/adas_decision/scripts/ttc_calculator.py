#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Time-to-Collision (TTC) Calculator
Based on steering angle path prediction

Features:
  - Predict vehicle path using steering angle (Ackermann model)
  - Auto-detect vehicle width from CARLA vehicle_info
  - Detect obstacles along predicted path
  - Lateral intrusion detection with path-aware range

Model: S + V_rel * TTC + 0.5 * a * TTC^2 = 0

Subscribed Topics:
    /adas/obstacles (adas_msgs/ObstacleArray)
    /carla/ego_vehicle/vehicle_status (carla_msgs/CarlaEgoVehicleStatus)
    /carla/ego_vehicle/vehicle_info (carla_msgs/CarlaEgoVehicleInfo)
    
Published Topics:
    /adas/ttc_info (adas_msgs/TTCInfo)
"""

from __future__ import annotations
import sys
import os

# Add carla-ros-bridge Python path for carla_msgs
carla_bridge_path = os.path.expanduser('~/carla-ros-bridge/catkin_ws/devel/lib/python3/dist-packages')
if os.path.exists(carla_bridge_path) and carla_bridge_path not in sys.path:
    sys.path.insert(0, carla_bridge_path)

import rospy
import numpy as np
from typing import List, Tuple

from adas_msgs.msg import ObstacleArray, TTCInfo
from carla_msgs.msg import CarlaEgoVehicleStatus, CarlaEgoVehicleInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class TTCCalculator:
    def __init__(self):
        rospy.init_node('ttc_calculator', anonymous=False)
        
        # Parameters
        self.min_ttc = rospy.get_param('~min_ttc', 0.1)
        self.max_ttc = rospy.get_param('~max_ttc', 10.0)
        self.safety_margin = rospy.get_param('~safety_margin', 2.0)
        
        # Corridor margin (added to auto-detected vehicle width)
        self.lane_margin = rospy.get_param('~lane_margin', 0.2)
        
        # Path prediction parameters
        self.path_points = rospy.get_param('~path_points', 20)
        self.path_max_distance = rospy.get_param('~path_max_distance', 50.0)
        
        # Lateral intrusion detection
        self.lateral_intrusion_enabled = rospy.get_param('~lateral_intrusion_enabled', True)
        self.lateral_detection_width = rospy.get_param('~lateral_detection_width', 3.0)
        self.lateral_velocity_threshold = rospy.get_param('~lateral_velocity_threshold', 0.5)
        self.intrusion_ttc_threshold = rospy.get_param('~intrusion_ttc_threshold', 2.0)
        
        # Suspended object filter
        self.sign_filter_enabled = rospy.get_param('~sign_filter_enabled', False)
        self.sign_min_ground_clearance = rospy.get_param('~sign_min_ground_clearance', 1.0)
        
        # Stability filtering
        self.min_ego_velocity = rospy.get_param('~min_ego_velocity', 0.3)
        self.stable_frames_required = rospy.get_param('~stable_frames_required', 1)
        
        # Vehicle parameters (auto-detected from vehicle_info)
        self.ego_width = 2.0  # Default, will be overwritten
        self.wheelbase = 3.0  # Default
        self.max_steer_angle = 1.22  # Default (rad)
        self.vehicle_info_received = False
        
        # LiDAR to front bumper offset (meters)
        # LiDAR is typically mounted on roof, vehicle front is ~2.5m ahead
        self.lidar_to_front_offset = rospy.get_param('~lidar_to_front_offset', 2.5)
        
        # State
        self.ego_velocity = 0.0
        self.ego_acceleration = 0.0
        self.steering_angle = 0.0  # Current wheel angle (rad)
        self.last_velocity = 0.0
        self.last_velocity_time = None
        
        # Obstacle stability tracking
        self.obstacle_history = {}
        
        # Debug
        self._log_counter = 0
        
        # Publishers
        self.ttc_pub = rospy.Publisher('/adas/ttc_info', TTCInfo, queue_size=1)
        self.path_pub = rospy.Publisher('/adas/predicted_path', MarkerArray, queue_size=1)
        
        # Subscribers
        self.obstacle_sub = rospy.Subscriber(
            '/adas/obstacles', ObstacleArray, self.obstacle_callback, queue_size=1
        )
        self.vehicle_status_sub = rospy.Subscriber(
            '/carla/ego_vehicle/vehicle_status', CarlaEgoVehicleStatus, 
            self.vehicle_status_callback, queue_size=1
        )
        self.vehicle_info_sub = rospy.Subscriber(
            '/carla/ego_vehicle/vehicle_info', CarlaEgoVehicleInfo,
            self.vehicle_info_callback, queue_size=1
        )
        
        rospy.loginfo("TTC Calculator initialized (path-based detection)")
        rospy.loginfo(f"  Lane margin: {self.lane_margin}m (added to auto-detected vehicle width)")
    
    def vehicle_info_callback(self, msg: CarlaEgoVehicleInfo):
        """Get vehicle geometry from CARLA"""
        if self.vehicle_info_received:
            return
        
        # Calculate vehicle width from wheel positions
        # CARLA wheel.position is in CENTIMETERS, need to convert to meters
        if len(msg.wheels) >= 2:
            # Check if values seem to be in cm (typically > 50 for normal vehicles)
            left_y = abs(msg.wheels[0].position.y)
            right_y = abs(msg.wheels[1].position.y)
            
            # If values > 10, assume centimeters and convert to meters
            if left_y > 10 or right_y > 10:
                left_y = left_y / 100.0
                right_y = right_y / 100.0
            
            # Width = wheel track + body overhang (0.4m each side for typical vehicle)
            self.ego_width = (left_y + right_y) + 0.8
        
        # Calculate wheelbase
        if len(msg.wheels) >= 4:
            front_x = msg.wheels[0].position.x
            rear_x = msg.wheels[2].position.x
            self.wheelbase = abs(front_x - rear_x)
        
        # Get max steering angle
        if len(msg.wheels) >= 1:
            self.max_steer_angle = msg.wheels[0].max_steer_angle
        
        self.vehicle_info_received = True
        
        corridor_half_width = (self.ego_width / 2.0) + self.lane_margin
        
        rospy.loginfo(f"Vehicle info received:")
        rospy.loginfo(f"  Type: {msg.type}")
        rospy.loginfo(f"  Width: {self.ego_width:.2f}m (auto-detected)")
        rospy.loginfo(f"  Wheelbase: {self.wheelbase:.2f}m")
        rospy.loginfo(f"  Max steer angle: {np.degrees(self.max_steer_angle):.1f} deg")
        rospy.loginfo(f"  Corridor half-width: {corridor_half_width:.2f}m")
    
    def vehicle_status_callback(self, msg: CarlaEgoVehicleStatus):
        """Get vehicle status from CARLA (velocity, steering)"""
        current_time = msg.header.stamp
        new_velocity = msg.velocity
        
        # Convert normalized steer [-1, 1] to actual wheel angle
        # CARLA: negative steer = right turn, positive = left turn
        # Ackermann model: positive steering = left turn (positive Y direction)
        # CARLA convention is OPPOSITE, so we negate the sign
        self.steering_angle = -msg.control.steer * self.max_steer_angle
        
        # Estimate acceleration
        if self.last_velocity_time is not None:
            dt = (current_time - self.last_velocity_time).to_sec()
            if dt > 0.01:
                self.ego_acceleration = (new_velocity - self.last_velocity) / dt
        
        self.last_velocity = new_velocity
        self.ego_velocity = new_velocity
        self.last_velocity_time = current_time
    
    def _predict_path(self, max_distance: float) -> List[Tuple[float, float]]:
        """
        Predict vehicle path using Ackermann steering model
        
        Turn radius R = wheelbase / tan(steering_angle)
        
        Returns:
            List of (x, y) points in vehicle local frame
        """
        path = []
        
        # Straight driving if steering angle is very small
        if abs(self.steering_angle) < 0.01:  # ~0.5 deg
            for i in range(self.path_points):
                x = (i + 1) * max_distance / self.path_points
                path.append((x, 0.0))
            return path
        
        # Ackermann turn radius
        turn_radius = self.wheelbase / np.tan(abs(self.steering_angle))
        
        # Turn direction: positive steering = left turn = positive y
        direction = np.sign(self.steering_angle)
        
        # Circle center at (0, direction * turn_radius)
        center_y = direction * turn_radius
        
        for i in range(self.path_points):
            arc_length = (i + 1) * max_distance / self.path_points
            theta = arc_length / turn_radius
            
            # Limit max angle
            if theta > np.pi / 2:
                theta = np.pi / 2
            
            # Point on arc
            x = turn_radius * np.sin(theta)
            y = center_y - direction * turn_radius * np.cos(theta)
            
            path.append((x, y))
        
        return path
    
    def _point_to_path_distance(self, px: float, py: float, path: List[Tuple[float, float]]) -> Tuple[float, float, int]:
        """
        Calculate shortest distance from point to predicted path
        
        Returns:
            (min_distance, along_path_distance, closest_segment_index)
        """
        min_dist = float('inf')
        along_path_dist = 0.0
        closest_idx = 0
        
        prev_x, prev_y = 0.0, 0.0
        cumulative_dist = 0.0
        
        for i, (x, y) in enumerate(path):
            seg_len = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
            
            if seg_len < 0.001:
                prev_x, prev_y = x, y
                continue
            
            # Point to line segment distance
            dx, dy = x - prev_x, y - prev_y
            t = max(0, min(1, ((px - prev_x) * dx + (py - prev_y) * dy) / (seg_len * seg_len)))
            
            closest_x = prev_x + t * dx
            closest_y = prev_y + t * dy
            
            dist = np.sqrt((px - closest_x)**2 + (py - closest_y)**2)
            
            if dist < min_dist:
                min_dist = dist
                along_path_dist = cumulative_dist + t * seg_len
                closest_idx = i
            
            cumulative_dist += seg_len
            prev_x, prev_y = x, y
        
        return min_dist, along_path_dist, closest_idx
    
    def _is_suspended_object(self, obstacle) -> bool:
        """Check if object is suspended (like overhead signs)"""
        if not self.sign_filter_enabled:
            return False
        
        height = obstacle.height
        obs_z = obstacle.position.z
        
        # Ground clearance (CARLA: z=0 is LiDAR height, ground at z=-2.4)
        ground_clearance = (obs_z - height / 2.0) + 2.4
        
        return ground_clearance > self.sign_min_ground_clearance
    
    def _get_corridor_half_width(self) -> float:
        """Get corridor half-width based on vehicle width"""
        return (self.ego_width / 2.0) + self.lane_margin
    
    def _publish_path_marker(self, path: List[Tuple[float, float]], corridor_half_width: float, stamp):
        """Publish predicted path as RViz MarkerArray for visualization"""
        marker_array = MarkerArray()
        
        # Path center line (green)
        center_marker = Marker()
        center_marker.header.frame_id = "adas_lidar"
        center_marker.header.stamp = stamp
        center_marker.ns = "predicted_path"
        center_marker.id = 0
        center_marker.type = Marker.LINE_STRIP
        center_marker.action = Marker.ADD
        center_marker.scale.x = 0.15  # Line width
        center_marker.color = ColorRGBA(0.0, 1.0, 0.0, 0.9)  # Green
        center_marker.pose.orientation.w = 1.0
        
        # Add origin point
        p = Point()
        p.x, p.y, p.z = 0.0, 0.0, 0.0
        center_marker.points.append(p)
        
        # Add path points
        for x, y in path:
            p = Point()
            p.x, p.y, p.z = x, y, 0.0
            center_marker.points.append(p)
        
        marker_array.markers.append(center_marker)
        
        # Left boundary (yellow)
        left_marker = Marker()
        left_marker.header.frame_id = "adas_lidar"
        left_marker.header.stamp = stamp
        left_marker.ns = "corridor"
        left_marker.id = 1
        left_marker.type = Marker.LINE_STRIP
        left_marker.action = Marker.ADD
        left_marker.scale.x = 0.08
        left_marker.color = ColorRGBA(1.0, 1.0, 0.0, 0.7)  # Yellow
        left_marker.pose.orientation.w = 1.0
        
        # Right boundary (yellow)
        right_marker = Marker()
        right_marker.header.frame_id = "adas_lidar"
        right_marker.header.stamp = stamp
        right_marker.ns = "corridor"
        right_marker.id = 2
        right_marker.type = Marker.LINE_STRIP
        right_marker.action = Marker.ADD
        right_marker.scale.x = 0.08
        right_marker.color = ColorRGBA(1.0, 1.0, 0.0, 0.7)  # Yellow
        right_marker.pose.orientation.w = 1.0
        
        prev_x, prev_y = 0.0, 0.0
        
        # Origin boundaries
        pl = Point()
        pl.x, pl.y, pl.z = 0.0, corridor_half_width, 0.0
        left_marker.points.append(pl)
        
        pr = Point()
        pr.x, pr.y, pr.z = 0.0, -corridor_half_width, 0.0
        right_marker.points.append(pr)
        
        for x, y in path:
            # Calculate perpendicular direction
            dx = x - prev_x
            dy = y - prev_y
            length = np.sqrt(dx*dx + dy*dy)
            if length > 0.001:
                nx = -dy / length
                ny = dx / length
            else:
                nx, ny = 0.0, 1.0
            
            # Left boundary point
            pl = Point()
            pl.x = x + nx * corridor_half_width
            pl.y = y + ny * corridor_half_width
            pl.z = 0.0
            left_marker.points.append(pl)
            
            # Right boundary point
            pr = Point()
            pr.x = x - nx * corridor_half_width
            pr.y = y - ny * corridor_half_width
            pr.z = 0.0
            right_marker.points.append(pr)
            
            prev_x, prev_y = x, y
        
        marker_array.markers.append(left_marker)
        marker_array.markers.append(right_marker)
        
        self.path_pub.publish(marker_array)
    
    def _calculate_ttc_second_order(self, s: float, v_rel: float, a_rel: float) -> float:
        """
        Calculate TTC using second-order kinematic model
        
        Solve: s - v_rel * t - 0.5 * a_rel * t^2 = 0
        """
        if v_rel <= 0:
            return float('inf')
        
        # First order (no deceleration)
        if abs(a_rel) < 0.1:
            return s / v_rel
        
        # Quadratic formula: 0.5*a*t^2 + v*t - s = 0
        a = 0.5 * a_rel
        b = v_rel
        c = -s
        
        discriminant = b*b - 4*a*c
        if discriminant < 0:
            return float('inf')
        
        sqrt_d = np.sqrt(discriminant)
        t1 = (-b + sqrt_d) / (2*a)
        t2 = (-b - sqrt_d) / (2*a)
        
        # Return smallest positive solution
        if t1 > 0 and t2 > 0:
            return min(t1, t2)
        elif t1 > 0:
            return t1
        elif t2 > 0:
            return t2
        else:
            return float('inf')
    
    def obstacle_callback(self, msg: ObstacleArray):
        """Calculate TTC for obstacles along predicted path"""
        ttc_msg = TTCInfo()
        ttc_msg.header = msg.header
        ttc_msg.ttc = -1.0
        ttc_msg.collision_imminent = False
        
        self._log_counter += 1
        
        # Predict vehicle path and publish for visualization
        path = self._predict_path(self.path_max_distance)
        corridor_half_width = self._get_corridor_half_width()
        
        # Always publish path for RViz visualization
        if self._log_counter % 5 == 0:  # 4Hz visualization
            self._publish_path_marker(path, corridor_half_width, msg.header.stamp)
        
        # Skip if velocity too low
        if abs(self.ego_velocity) < self.min_ego_velocity:
            if self._log_counter % 100 == 0:
                rospy.loginfo(f"TTC skip: velocity {self.ego_velocity:.2f} < {self.min_ego_velocity}")
            self.ttc_pub.publish(ttc_msg)
            return
        
        if len(msg.obstacles) == 0:
            self.ttc_pub.publish(ttc_msg)
            return
        
        # Debug logging (every ~0.5s at 20Hz)
        if self._log_counter % 10 == 0:
            steer_deg = np.degrees(self.steering_angle)
            rospy.loginfo(f"Obstacles: {len(msg.obstacles)}, steer={steer_deg:.1f}deg, v={self.ego_velocity:.1f}m/s, corridor={corridor_half_width:.2f}m, ego_width={self.ego_width:.2f}m")
        
        # Update obstacle stability counts
        current_ids = set()
        for obs in msg.obstacles:
            current_ids.add(obs.id)
            self.obstacle_history[obs.id] = self.obstacle_history.get(obs.id, 0) + 1
        
        # Remove disappeared obstacles
        for oid in list(self.obstacle_history.keys()):
            if oid not in current_ids:
                del self.obstacle_history[oid]
        
        # Find most dangerous obstacle
        best_obstacle = None
        best_ttc = float('inf')
        best_reason = ""
        
        for obstacle in msg.obstacles:
            # Check stability
            if self.obstacle_history.get(obstacle.id, 0) < self.stable_frames_required:
                continue
            
            obs_x = obstacle.position.x
            obs_y = obstacle.position.y
            obs_vx = obstacle.velocity.x if hasattr(obstacle.velocity, 'x') else 0.0
            obs_vy = obstacle.velocity.y if hasattr(obstacle.velocity, 'y') else 0.0
            
            # Only consider obstacles in front
            if obs_x < 0.5:
                continue
            
            # Filter suspended objects
            if self._is_suspended_object(obstacle):
                continue
            
            # Calculate distance to predicted path
            dist_to_path, along_path_dist, _ = self._point_to_path_distance(obs_x, obs_y, path)
            
            # Debug: log obstacle info
            if self._log_counter % 20 == 0:
                rospy.loginfo(f"  Obs {obstacle.id}: x={obs_x:.1f}, y={obs_y:.1f}, dist_to_path={dist_to_path:.2f}, corridor={corridor_half_width:.2f}")
            
            # Check if obstacle is within corridor along the path
            in_corridor = dist_to_path <= corridor_half_width
            
            # Lateral intrusion detection (for obstacles outside corridor)
            is_intruding = False
            intrusion_ttc = float('inf')
            
            if self.lateral_intrusion_enabled and not in_corridor:
                # Dynamic lateral detection range based on path curvature
                # When turning, reduce lateral detection on the outer side
                lateral_range = self.lateral_detection_width
                
                if abs(self.steering_angle) > 0.05:
                    # Reduce detection on the outer side of the turn
                    if (self.steering_angle > 0 and obs_y < 0) or (self.steering_angle < 0 and obs_y > 0):
                        lateral_range *= 0.5  # Reduce outer side detection
                
                if dist_to_path <= lateral_range:
                    # Calculate lateral approach velocity toward the path
                    # Need to project velocity toward the path center
                    lateral_approach_velocity = abs(obs_vy)  # Simplified
                    
                    if lateral_approach_velocity > self.lateral_velocity_threshold:
                        distance_to_corridor = dist_to_path - corridor_half_width
                        if distance_to_corridor > 0:
                            intrusion_ttc = distance_to_corridor / lateral_approach_velocity
                            
                            if intrusion_ttc < self.intrusion_ttc_threshold:
                                is_intruding = True
                                if self._log_counter % 30 == 0:
                                    side = "left" if obs_y > 0 else "right"
                                    rospy.logwarn(f"Lateral intrusion: {side}, vy={obs_vy:.1f}m/s, in {intrusion_ttc:.1f}s")
            
            # Skip if not in corridor and not intruding
            if not in_corridor and not is_intruding:
                continue
            
            # Calculate TTC
            v_rel = self.ego_velocity - obs_vx
            if v_rel < 0.1:  # Lower threshold
                continue
            
            # Calculate actual collision distance:
            # obstacle.distance = LiDAR to obstacle centroid
            # Need to subtract: LiDAR-to-front offset + obstacle half-length
            actual_gap = obstacle.distance - self.lidar_to_front_offset - (obstacle.length / 2.0) - self.safety_margin
            
            if actual_gap <= 0:
                ttc = 0.0
                if self._log_counter % 10 == 0:
                    rospy.logwarn(f"  COLLISION IMMINENT: gap={actual_gap:.2f}m (raw_dist={obstacle.distance:.1f}, lidar_offset={self.lidar_to_front_offset:.1f}, obs_len={obstacle.length:.1f})")
            else:
                ttc = self._calculate_ttc_second_order(actual_gap, v_rel, -self.ego_acceleration)
            
            # Debug: log TTC calculation
            if self._log_counter % 10 == 0 and in_corridor:
                rospy.loginfo(f"  -> IN CORRIDOR: raw_dist={obstacle.distance:.1f}m, actual_gap={actual_gap:.1f}m, v_rel={v_rel:.1f}m/s, TTC={ttc:.2f}s")
            
            if is_intruding:
                effective_ttc = min(ttc if ttc > 0 else float('inf'), intrusion_ttc)
                reason = "lateral_intrusion"
            else:
                effective_ttc = ttc
                reason = "in_path"
            
            if 0 < effective_ttc < best_ttc:
                best_ttc = effective_ttc
                best_obstacle = obstacle
                best_reason = reason
        
        # Fill message
        if best_obstacle is not None and best_ttc < self.max_ttc:
            ttc_msg.ttc = best_ttc
            ttc_msg.collision_imminent = (0 < best_ttc < 0.6)
            ttc_msg.distance = best_obstacle.distance
            ttc_msg.relative_velocity = self.ego_velocity - (best_obstacle.velocity.x if hasattr(best_obstacle.velocity, 'x') else 0.0)
            ttc_msg.relative_acceleration = -self.ego_acceleration
            ttc_msg.confidence = best_obstacle.confidence
            
            if self._log_counter % 20 == 0:
                rospy.loginfo(f"TTC: {best_ttc:.2f}s, dist={best_obstacle.distance:.1f}m, reason={best_reason}")
        
        self.ttc_pub.publish(ttc_msg)
    
    def run(self):
        rospy.spin()


def main():
    try:
        calculator = TTCCalculator()
        calculator.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
