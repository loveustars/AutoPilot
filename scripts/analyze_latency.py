#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Latency Analyzer - Measure end-to-end system latency
延迟分析工具 - 测量端到端系统延迟

Usage: python3 analyze_latency.py
"""

import rospy
import numpy as np
from collections import deque
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import AccelStamped


class LatencyAnalyzer:
    def __init__(self):
        rospy.init_node('latency_analyzer', anonymous=True)
        
        self.lidar_times = deque(maxlen=100)
        self.command_times = deque(maxlen=100)
        self.latencies = deque(maxlen=100)
        
        self.lidar_sub = rospy.Subscriber('/adas/lidar', PointCloud2, self.lidar_cb)
        self.cmd_sub = rospy.Subscriber('/adas/safe_command', AccelStamped, self.cmd_cb)
        
        self.timer = rospy.Timer(rospy.Duration(5.0), self.report)
        
        rospy.loginfo("Latency Analyzer started")
    
    def lidar_cb(self, msg):
        self.lidar_times.append(msg.header.stamp)
    
    def cmd_cb(self, msg):
        now = rospy.Time.now()
        if self.lidar_times:
            # Estimate latency from most recent LiDAR
            latency = (now - self.lidar_times[-1]).to_sec() * 1000
            self.latencies.append(latency)
    
    def report(self, event):
        if not self.latencies:
            rospy.loginfo("No latency data yet...")
            return
        
        latencies = np.array(self.latencies)
        rospy.loginfo("=" * 50)
        rospy.loginfo("LATENCY REPORT (last 100 samples)")
        rospy.loginfo(f"  Mean:   {np.mean(latencies):.1f} ms")
        rospy.loginfo(f"  Std:    {np.std(latencies):.1f} ms")
        rospy.loginfo(f"  Min:    {np.min(latencies):.1f} ms")
        rospy.loginfo(f"  Max:    {np.max(latencies):.1f} ms")
        rospy.loginfo(f"  P95:    {np.percentile(latencies, 95):.1f} ms")
        rospy.loginfo(f"  Target: <100 ms {'✓' if np.percentile(latencies, 95) < 100 else '✗'}")
        rospy.loginfo("=" * 50)


if __name__ == '__main__':
    try:
        analyzer = LatencyAnalyzer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
