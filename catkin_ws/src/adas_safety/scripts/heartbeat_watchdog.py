#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Heartbeat Watchdog - Standalone lightweight watchdog
轻量级独立心跳监控 (备用)

This is a simplified watchdog that only monitors heartbeat.
Use safety_monitor.py for full functionality.
"""

import rospy
from std_msgs.msg import Header, Bool


class HeartbeatWatchdog:
    def __init__(self):
        rospy.init_node('heartbeat_watchdog', anonymous=False)
        
        self.timeout_ms = rospy.get_param('~timeout_ms', 50.0)
        self.last_heartbeat = rospy.Time.now()
        self.alive = True
        
        self.status_pub = rospy.Publisher('/adas/watchdog_status', Bool, queue_size=1)
        self.heartbeat_sub = rospy.Subscriber('/adas/heartbeat', Header, self.heartbeat_cb, queue_size=1)
        
        self.timer = rospy.Timer(rospy.Duration(0.01), self.check_timeout)
        
        rospy.loginfo(f"Heartbeat Watchdog: timeout={self.timeout_ms}ms")
    
    def heartbeat_cb(self, msg):
        self.last_heartbeat = msg.stamp
        if not self.alive:
            rospy.loginfo("Watchdog: Node recovered")
        self.alive = True
    
    def check_timeout(self, event):
        age_ms = (rospy.Time.now() - self.last_heartbeat).to_sec() * 1000
        was_alive = self.alive
        self.alive = age_ms <= self.timeout_ms
        
        if was_alive and not self.alive:
            rospy.logerr(f"Watchdog: TIMEOUT detected ({age_ms:.0f}ms)")
        
        self.status_pub.publish(Bool(data=self.alive))
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = HeartbeatWatchdog()
        node.run()
    except rospy.ROSInterruptException:
        pass
