#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiDAR Point Cloud Processor
处理LiDAR点云数据：体素滤波、地面去除、DBSCAN聚类

Subscribed Topics:
    /adas/lidar (sensor_msgs/PointCloud2)
    
Published Topics:
    /adas/obstacles (adas_msgs/ObstacleArray)
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN

from adas_msgs.msg import Obstacle, ObstacleArray


class LidarProcessor:
    def __init__(self):
        rospy.init_node('lidar_processor', anonymous=False)
        
        # Parameters
        self.voxel_size = rospy.get_param('~voxel_size', 0.15)  # meters
        self.ground_z_max = rospy.get_param('~ground_z_max', -1.5)  # CARLA ground level (adjusted)
        self.cluster_eps = rospy.get_param('~cluster_eps', 0.8)
        self.cluster_min_samples = rospy.get_param('~cluster_min_samples', 5)
        self.max_distance = rospy.get_param('~max_distance', 50.0)
        
        # ROI limits - 关键参数防止误检测
        self.min_x = rospy.get_param('~min_x', 2.0)   # 前方 - 2m避免车身噪声
        self.max_x = rospy.get_param('~max_x', 50.0)
        self.min_y = rospy.get_param('~min_y', -6.0)  # 左右 - ±6m
        self.max_y = rospy.get_param('~max_y', 6.0)
        self.min_z = rospy.get_param('~min_z', -1.0)  # 高度 - 只保留离地1.4m以上
        self.max_z = rospy.get_param('~max_z', 1.5)
        
        # Minimum obstacle size requirements
        self.min_obstacle_points = rospy.get_param('~min_obstacle_points', 10)  # 10点
        self.min_obstacle_size = rospy.get_param('~min_obstacle_size', 0.3)  # 0.3m
        
        # Publishers
        self.obstacle_pub = rospy.Publisher(
            '/adas/obstacles', ObstacleArray, queue_size=1
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
        """Filter points to ROI and remove ground (adapted for CARLA)
        
        CARLA LiDAR坐标系:
        - X: 前方为正
        - Y: 左侧为正  
        - Z: 上方为正，地面约在 z=-2.4 (LiDAR安装高度)
        """
        if len(points) == 0:
            return points
        
        # ROI过滤：只保留前方、合理高度范围内的点
        mask = (
            (points[:, 0] > self.min_x) &  # 前方
            (points[:, 0] < self.max_x) &
            (points[:, 1] > self.min_y) &  # 左右
            (points[:, 1] < self.max_y) &
            (points[:, 2] > self.min_z) &  # 高于地面
            (points[:, 2] < self.max_z)    # 低于天空
        )
        return points[mask]
    
    def _cluster_obstacles(self, points: np.ndarray) -> list:
        """Cluster points into obstacles using DBSCAN"""
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
            if label == -1:  # Noise
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
            
            # Filter small obstacles (likely noise)
            if len(cluster_points) < self.min_obstacle_points:
                continue
            if max(width, length) < self.min_obstacle_size:
                continue
            
            obstacles.append({
                'id': self._get_next_id(),
                'centroid': centroid,
                'distance': distance,
                'width': width,
                'length': length,
                'height': max_pt[2] - min_pt[2],
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
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = LidarProcessor()
        node.run()
    except rospy.ROSInterruptException:
        pass
