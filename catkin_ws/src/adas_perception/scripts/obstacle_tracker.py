#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Obstacle Tracker
跟踪障碍物并估计速度

Subscribed Topics:
    /adas/obstacles (adas_msgs/ObstacleArray)
    /adas/ego_state (geometry_msgs/TwistStamped) - ego vehicle velocity
    
Published Topics:
    /adas/tracked_obstacles (adas_msgs/ObstacleArray)
"""
from __future__ import annotations

import rospy
import numpy as np
from collections import deque
from typing import Dict
from geometry_msgs.msg import TwistStamped

from adas_msgs.msg import Obstacle, ObstacleArray


class TrackedObject:
    """Single tracked obstacle with history"""
    def __init__(self, obstacle: Obstacle, max_history: int = 10):
        self.id = obstacle.id
        self.history = deque(maxlen=max_history)
        self.history.append({
            'time': rospy.Time.now(),
            'position': np.array([obstacle.position.x, obstacle.position.y, obstacle.position.z]),
            'distance': obstacle.distance
        })
        self.velocity = np.zeros(3)
        self.lost_count = 0
        self.classification = obstacle.classification
        
    def update(self, obstacle: Obstacle):
        """Update with new detection"""
        current_time = rospy.Time.now()
        current_pos = np.array([obstacle.position.x, obstacle.position.y, obstacle.position.z])
        
        # Calculate velocity from history
        if len(self.history) > 0:
            last = self.history[-1]
            dt = (current_time - last['time']).to_sec()
            if dt > 0.01:  # Avoid division by near-zero
                self.velocity = (current_pos - last['position']) / dt
        
        self.history.append({
            'time': current_time,
            'position': current_pos,
            'distance': obstacle.distance
        })
        self.lost_count = 0
        self.classification = obstacle.classification
        
    def predict(self, dt: float) -> np.ndarray:
        """Predict position after dt seconds"""
        if len(self.history) > 0:
            return self.history[-1]['position'] + self.velocity * dt
        return np.zeros(3)
    
    def get_relative_velocity(self) -> float:
        """Get velocity component towards ego (positive = approaching)"""
        if len(self.history) > 0:
            pos = self.history[-1]['position']
            direction = -pos / (np.linalg.norm(pos[:2]) + 1e-6)
            return np.dot(self.velocity[:2], direction[:2])
        return 0.0


class ObstacleTracker:
    def __init__(self):
        rospy.init_node('obstacle_tracker', anonymous=False)
        
        # Parameters
        self.association_threshold = rospy.get_param('~association_threshold', 2.0)
        self.max_lost_frames = rospy.get_param('~max_lost_frames', 5)
        
        # Tracked objects
        self.tracked_objects: Dict[int, TrackedObject] = {}
        self.next_track_id = 1
        
        # Ego velocity
        self.ego_velocity = np.zeros(3)
        
        # Publishers
        self.tracked_pub = rospy.Publisher(
            '/adas/tracked_obstacles', ObstacleArray, queue_size=1
        )
        
        # Subscribers
        self.obstacle_sub = rospy.Subscriber(
            '/adas/obstacles', ObstacleArray, self.obstacle_callback, queue_size=1
        )
        self.ego_sub = rospy.Subscriber(
            '/adas/ego_state', TwistStamped, self.ego_callback, queue_size=1
        )
        
        rospy.loginfo("Obstacle Tracker initialized")
    
    def ego_callback(self, msg: TwistStamped):
        """Update ego velocity"""
        self.ego_velocity = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ])
    
    def obstacle_callback(self, msg: ObstacleArray):
        """Process obstacle detections and update tracks"""
        # Mark all tracks as potentially lost
        for track in self.tracked_objects.values():
            track.lost_count += 1
        
        # Associate detections to tracks
        for obstacle in msg.obstacles:
            matched_id = self._find_matching_track(obstacle)
            
            if matched_id is not None:
                self.tracked_objects[matched_id].update(obstacle)
            else:
                # Create new track
                new_track = TrackedObject(obstacle)
                new_track.id = self.next_track_id
                self.tracked_objects[self.next_track_id] = new_track
                self.next_track_id += 1
        
        # Remove lost tracks
        lost_ids = [
            tid for tid, track in self.tracked_objects.items()
            if track.lost_count > self.max_lost_frames
        ]
        for tid in lost_ids:
            del self.tracked_objects[tid]
        
        # Publish tracked obstacles
        self._publish_tracks(msg.header)
    
    def _find_matching_track(self, obstacle: Obstacle) -> int:
        """Find matching track for obstacle using nearest neighbor"""
        obs_pos = np.array([obstacle.position.x, obstacle.position.y])
        min_dist = float('inf')
        matched_id = None
        
        for tid, track in self.tracked_objects.items():
            if len(track.history) > 0:
                # Predict where track should be
                predicted = track.predict(0.1)[:2]
                dist = np.linalg.norm(obs_pos - predicted)
                
                if dist < min_dist and dist < self.association_threshold:
                    min_dist = dist
                    matched_id = tid
        
        return matched_id
    
    def _publish_tracks(self, header):
        """Publish tracked obstacles with velocity"""
        msg = ObstacleArray()
        msg.header = header
        msg.closest_obstacle_idx = -1
        
        min_distance = float('inf')
        
        for i, track in enumerate(self.tracked_objects.values()):
            if len(track.history) == 0:
                continue
                
            obstacle = Obstacle()
            obstacle.header = header
            obstacle.id = track.id
            obstacle.classification = track.classification
            
            pos = track.history[-1]['position']
            obstacle.position.x = pos[0]
            obstacle.position.y = pos[1]
            obstacle.position.z = pos[2]
            
            # Relative velocity (track velocity - ego velocity)
            rel_vel = track.velocity - self.ego_velocity
            obstacle.velocity.x = rel_vel[0]
            obstacle.velocity.y = rel_vel[1]
            obstacle.velocity.z = rel_vel[2]
            
            obstacle.distance = track.history[-1]['distance']
            
            msg.obstacles.append(obstacle)
            
            if obstacle.distance < min_distance:
                min_distance = obstacle.distance
                msg.closest_obstacle_idx = i
        
        self.tracked_pub.publish(msg)
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = ObstacleTracker()
        node.run()
    except rospy.ROSInterruptException:
        pass
