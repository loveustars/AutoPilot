#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Brake Actuator - Low-level brake control
底层制动控制

Handles brake actuation dynamics and limits.
"""

import rospy
import numpy as np
from geometry_msgs.msg import AccelStamped
from std_msgs.msg import Float32


class BrakeActuator:
    def __init__(self):
        rospy.init_node('brake_actuator', anonymous=False)
        
        # Parameters
        self.max_brake_rate = rospy.get_param('~max_brake_rate', 5.0)  # per second
        self.brake_delay = rospy.get_param('~brake_delay', 0.05)  # seconds
        
        # State
        self.target_brake = 0.0
        self.current_brake = 0.0
        self.last_update_time = rospy.Time.now()
        
        # Publishers
        self.brake_pub = rospy.Publisher('/adas/brake_output', Float32, queue_size=1)
        
        # Subscribers
        self.cmd_sub = rospy.Subscriber(
            '/adas/brake_cmd', Float32, self.brake_callback, queue_size=1
        )
        
        # Update timer
        self.timer = rospy.Timer(rospy.Duration(0.01), self.update)
        
        rospy.loginfo("Brake Actuator initialized")
    
    def brake_callback(self, msg: Float32):
        """Receive brake command"""
        self.target_brake = np.clip(msg.data, 0.0, 1.0)
    
    def update(self, event):
        """Update brake output with rate limiting"""
        now = rospy.Time.now()
        dt = (now - self.last_update_time).to_sec()
        self.last_update_time = now
        
        # Rate limit
        max_change = self.max_brake_rate * dt
        error = self.target_brake - self.current_brake
        change = np.clip(error, -max_change, max_change)
        self.current_brake += change
        
        # Publish
        self.brake_pub.publish(Float32(data=self.current_brake))
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = BrakeActuator()
        node.run()
    except rospy.ROSInterruptException:
        pass
