#! /usr/bin/env python3

import rospy
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

move = Twist()
move.linear.x = 0.5
move.angular.z = 0.0

minfrontdistance = 0.3
stopdist = 0.3
stop = 0
obstacle = False
speed = 0.5
AVOID_SPEED = 0.05

minleft = 10000000.0
minright = 10000000.0
maxleft = -1
maxright = -1

rospy.init_node("robot_0")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
rate = rospy.Rate(10)


def rotateRobot(direction):
    pass


def obstacleroutine(minleft, minright):
    pass


def callback(msg):
    pass


def rotate():
    global obstacle

    velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    vel_msg = Twist()

    print("Let's rotate your robot")
    clockwise = True  # input("Clockwise?: ") #True or false

    # Converting from angles to radians
    angular_speed = 20.0 * 2 * 3.14 / 360
    relative_angle = 90.0 * 2 * 3.14 / 360

    vel_msg.linear.x = 0
    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    r = rospy.Rate(10)
    while obstacle != False:
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed * (t1 - t0)
        r.sleep()

    # Forcing our robot to stop
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)


def controlrobot():
    pass


# sub = rospy.Subscriber('/base_scan', LaserScan, callback)
sub = rospy.Subscriber("/scan", LaserScan, callback)
controlrobot()
rospy.spin()
