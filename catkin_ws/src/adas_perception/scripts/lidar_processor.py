#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiDAR Point Cloud Processor
处理LiDAR点云数据：体素滤波、地面去除、DBSCAN聚类

Subscribed Topics:
    /adas/lidar (sensor_msgs/PointCloud2)
    
Published Topics:
    /adas/obstacles (adas_msgs/ObstacleArray)
    /adas/obstacle_markers (visualization_msgs/MarkerArray)
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

from adas_msgs.msg import Obstacle, ObstacleArray


class LidarProcessor:
    def __init__(self):
        rospy.init_node('lidar_processor', anonymous=False)
        
        # Parameters - Simplified for maximum obstacle detection
        # CARLA LiDAR is mounted at ~2.4m height
        # z = 0 is LiDAR position, ground is at z ≈ -2.4
        self.voxel_size = rospy.get_param('~voxel_size', 0.08)  # Small voxel for fine detail
        self.ground_z_threshold = rospy.get_param('~ground_z_threshold', -2.2)  # Ground plane (0.2m above actual ground)
        self.cluster_eps = rospy.get_param('~cluster_eps', 0.4)  # Tight clustering
        self.cluster_min_samples = rospy.get_param('~cluster_min_samples', 2)  # Minimal points
        self.max_distance = rospy.get_param('~max_distance', 50.0)
        
        # ROI limits - Wide detection zone
        # Keep everything above vehicle chassis clearance (~0.2m from ground = z > -2.2)
        self.min_x = rospy.get_param('~min_x', 1.5)   # Forward - 1.5m to avoid ego vehicle points
        self.max_x = rospy.get_param('~max_x', 60.0)  # Extended range
        self.min_y = rospy.get_param('~min_y', -6.0)  # Left/Right - wider for turns
        self.max_y = rospy.get_param('~max_y', 6.0)
        self.min_z = rospy.get_param('~min_z', -2.2)  # Above chassis clearance (0.2m from ground)
        self.max_z = rospy.get_param('~max_z', 2.0)   # Below sky
        
        # Minimum obstacle requirements - Very permissive
        self.min_obstacle_points = rospy.get_param('~min_obstacle_points', 2)  # Just 2 points
        self.min_obstacle_size = rospy.get_param('~min_obstacle_size', 0.05)  # 5cm minimum
        
        # Publishers
        self.obstacle_pub = rospy.Publisher(
            '/adas/obstacles', ObstacleArray, queue_size=1
        )
        self.marker_pub = rospy.Publisher(
            '/adas/obstacle_markers', MarkerArray, queue_size=1
        )
        
        # Subscribers
        self.lidar_sub = rospy.Subscriber(
            '/adas/lidar', PointCloud2, self.lidar_callback, queue_size=1
        )
        
        # Obstacle tracking
        self.obstacle_id_counter = 0
        
        rospy.loginfo("LiDAR Processor initialized")
    
    def lidar_callback(self, msg: PointCloud2):
        """Process incoming point cloud"""
        start_time = rospy.Time.now()
        
        # Convert to numpy array
        points = self._pointcloud2_to_numpy(msg)
        
        if len(points) == 0:
            return
        
        # Pipeline: Filter -> Ground Removal -> Clustering
        points_filtered = self._voxel_filter(points)
        points_no_ground = self._remove_ground(points_filtered)
        obstacles = self._cluster_obstacles(points_no_ground)
        
        # Publish
        obstacle_msg = self._create_obstacle_array(obstacles, msg.header)
        self.obstacle_pub.publish(obstacle_msg)
        
        # Publish visualization markers
        marker_msg = self._create_marker_array(obstacles, msg.header)
        self.marker_pub.publish(marker_msg)
        
        # Performance logging
        elapsed = (rospy.Time.now() - start_time).to_sec() * 1000
        if elapsed > 30:
            rospy.logwarn(f"LiDAR processing took {elapsed:.1f}ms (target: <30ms)")
    
    def _pointcloud2_to_numpy(self, cloud_msg: PointCloud2) -> np.ndarray:
        """Convert PointCloud2 to numpy array"""
        points = []
        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        return np.array(points)
    
    def _voxel_filter(self, points: np.ndarray) -> np.ndarray:
        """Apply voxel grid downsampling"""
        if len(points) == 0:
            return points
        
        # Simple voxel grid filter
        voxel_indices = np.floor(points / self.voxel_size).astype(int)
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        return points[unique_indices]
    
    def _remove_ground(self, points: np.ndarray) -> np.ndarray:
        """Filter points to ROI - keep everything above chassis clearance
        
        CARLA LiDAR coordinate system:
        - X: Forward positive
        - Y: Left positive  
        - Z: Up positive, ground at z ≈ -2.4 (LiDAR height)
        
        Strategy: Keep all points above ground (z > -2.2) within ROI.
        This includes ALL obstacles: barriers, poles, vehicles, pedestrians, etc.
        """
        if len(points) == 0:
            return points
        
        # Simple ROI filter - keep everything above chassis clearance
        mask = (
            (points[:, 0] > self.min_x) &  # Forward
            (points[:, 0] < self.max_x) &
            (points[:, 1] > self.min_y) &  # Left/Right
            (points[:, 1] < self.max_y) &
            (points[:, 2] > self.min_z) &  # Above ground (chassis clearance)
            (points[:, 2] < self.max_z)    # Below sky
        )
        
        filtered = points[mask]
        
        # Debug: log point counts occasionally
        if hasattr(self, '_filter_log_count'):
            self._filter_log_count += 1
        else:
            self._filter_log_count = 0
        
        if self._filter_log_count % 100 == 0:
            rospy.loginfo(f"LiDAR: {len(points)} raw -> {len(filtered)} after ROI filter")
        
        return filtered
    
    def _cluster_obstacles(self, points: np.ndarray) -> list:
        """Cluster points into obstacles using DBSCAN
        
        Strategy: Treat ALL clusters as potential obstacles.
        Let the path-based filtering in TTC calculator decide what's dangerous.
        """
        if len(points) < self.cluster_min_samples:
            return []
        
        clustering = DBSCAN(
            eps=self.cluster_eps,
            min_samples=self.cluster_min_samples
        ).fit(points)
        
        obstacles = []
        labels = clustering.labels_
        unique_labels = set(labels)
        
        for label in unique_labels:
            if label == -1:  # Noise points - still consider if enough points
                continue
            
            cluster_mask = labels == label
            cluster_points = points[cluster_mask]
            
            # Calculate obstacle properties
            centroid = np.mean(cluster_points, axis=0)
            distance = np.linalg.norm(centroid[:2])  # XY distance
            
            if distance > self.max_distance:
                continue
            
            # Bounding box
            min_pt = np.min(cluster_points, axis=0)
            max_pt = np.max(cluster_points, axis=0)
            
            width = max_pt[1] - min_pt[1]
            length = max_pt[0] - min_pt[0]
            height = max_pt[2] - min_pt[2]
            
            # Very minimal filtering - only reject truly tiny noise
            # Keep everything that could possibly be an obstacle
            if len(cluster_points) < 2:
                continue
            
            # Accept obstacle
            obstacles.append({
                'id': self._get_next_id(),
                'centroid': centroid,
                'distance': distance,
                'width': max(width, 0.1),  # Minimum 10cm for visualization
                'length': max(length, 0.1),
                'height': max(height, 0.1),
                'points': len(cluster_points)
            })
        
        return obstacles
    
    def _get_next_id(self) -> int:
        """Get next obstacle ID"""
        self.obstacle_id_counter += 1
        return self.obstacle_id_counter
    
    def _create_obstacle_array(self, obstacles: list, header) -> ObstacleArray:
        """Create ObstacleArray message"""
        msg = ObstacleArray()
        msg.header = header
        msg.closest_obstacle_idx = -1
        
        min_distance = float('inf')
        
        for i, obs in enumerate(obstacles):
            obstacle = Obstacle()
            obstacle.header = header
            obstacle.id = obs['id']
            obstacle.classification = Obstacle.UNKNOWN
            obstacle.position.x = obs['centroid'][0]
            obstacle.position.y = obs['centroid'][1]
            obstacle.position.z = obs['centroid'][2]
            obstacle.distance = obs['distance']
            obstacle.width = obs['width']
            obstacle.length = obs['length']
            obstacle.height = obs['height']
            # 修复置信度计算：基于点数，但使用更合理的基准
            # 5点=0.5, 10点=0.7, 20点=0.9, 50点+=1.0
            obstacle.confidence = min(1.0, 0.3 + obs['points'] / 30.0)
            
            msg.obstacles.append(obstacle)
            
            if obs['distance'] < min_distance:
                min_distance = obs['distance']
                msg.closest_obstacle_idx = i
        
        return msg
    
    def _create_marker_array(self, obstacles: list, header) -> MarkerArray:
        """Create MarkerArray for RViz visualization"""
        marker_array = MarkerArray()
        
        # First, add a delete all marker to clear old markers
        delete_marker = Marker()
        delete_marker.header = header
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        for i, obs in enumerate(obstacles):
            # Bounding box marker
            marker = Marker()
            marker.header = header
            marker.ns = "obstacles"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = obs['centroid'][0]
            marker.pose.position.y = obs['centroid'][1]
            marker.pose.position.z = obs['centroid'][2]
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = max(obs['length'], 0.5)
            marker.scale.y = max(obs['width'], 0.5)
            marker.scale.z = max(obs['height'], 0.5)
            
            # Color based on distance (red=close, green=far)
            dist_normalized = min(obs['distance'] / 30.0, 1.0)
            marker.color = ColorRGBA(
                r=1.0 - dist_normalized,
                g=dist_normalized,
                b=0.2,
                a=0.7
            )
            
            marker.lifetime = rospy.Duration(0.2)  # 200ms lifetime
            marker_array.markers.append(marker)
            
            # Text label with distance
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "obstacle_labels"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = obs['centroid'][0]
            text_marker.pose.position.y = obs['centroid'][1]
            text_marker.pose.position.z = obs['centroid'][2] + obs['height'] / 2 + 0.5
            
            text_marker.scale.z = 0.5  # Text height
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f"{obs['distance']:.1f}m"
            text_marker.lifetime = rospy.Duration(0.2)
            
            marker_array.markers.append(text_marker)
        
        return marker_array
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = LidarProcessor()
        node.run()
    except rospy.ROSInterruptException:
        pass
