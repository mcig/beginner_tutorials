#! /usr/bin/env python3

import sys
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

nodeid = str(sys.argv[1])
multipleBots = int(sys.argv[2])
nodename = f"/tb3_{nodeid}"

vel_msg = Twist()
vel_msg.linear.x = 0.5

print(f"Publishing to: {nodename}/cmd_vel" if multipleBots else "Publishing to: /cmd_vel")
print(f"Scanning from: {nodename}/scan" if multipleBots else "Scanning from /scan")

def scan_callback(msg):
    hasObstacleInFront = False
    for i in range(len(msg.ranges)):
        leftFront = i > 335 and i < 359
        rightFront = i > 0 and i < 25
        currentDistance = msg.ranges[i]
        if (leftFront or rightFront) and currentDistance < 0.5:
            hasObstacleInFront = True
            break

    if hasObstacleInFront:
        vel_msg.linear.x = 0.0
    else:
        vel_msg.linear.x = 0.5
        
    pub.publish(vel_msg)

rospy.init_node('bot_scan_forward', anonymous=True)
pub = rospy.Publisher(f"{nodename}/cmd_vel" if multipleBots else "/cmd_vel", Twist, queue_size=10)
sub = rospy.Subscriber(f"{nodename}/scan" if multipleBots else "/scan", LaserScan, scan_callback)
rospy.spin()

