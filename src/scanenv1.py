#! /usr/bin/env python3

import sys
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

nodeid = str(sys.argv[1])
multipleBots = int(sys.argv[2])
nodename = f"/robot_{nodeid}"

vel_msg = Twist()
vel_msg.linear.x = 0.5


def scan_callback(msg):
    hasObstacle = False
    for i in range(len(msg.ranges)):
        print(f"Range {i}: {msg.ranges[i]}")
        pass

    midPoint = len(msg.ranges) // 2
    if msg.ranges[midPoint] > 0.5:
        vel_msg.linear.x = 0.5
    else:
        vel_msg.linear.x = 0.0
        
    pub.publish(vel_msg)

rospy.init_node('bot_scan_forward', anonymous=True)
pub = rospy.Publisher(f"{nodename}/cmd_vel" if multipleBots else "/cmd_vel", Twist, queue_size=10)
sub = rospy.Subscriber(f"{nodename}/base_scan" if multipleBots else "/base_scan", LaserScan, scan_callback)
rospy.spin()

