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

nodeid = str(sys.argv[1])
nodename = f"/tb3_{nodeid}"

rospy.init_node("wallFollow")
pub = rospy.Publisher(nodename + "/cmd_vel", Twist, queue_size=10)
rate = rospy.Rate(40)


def rotateRobot(direction):
    move.linear.x = 0.0
    move.angular.z  = 0.1 * direction
    pub.publish(move)


def obstacleroutine(minleft, minright):
    if(stop == 1):
        speed = AVOID_SPEED

    if(minleft < minright):
        rotateRobot(1)
    else:
        rotateRobot(-1)


def callback(msg):
    global obstacle
    obstacle = False
    size = len(msg.ranges)
    global minleft
    global minright
    global maxleft
    global maxright

    minlid = minrid = 0
    for i in range(0, 30):
        if(float(msg.ranges[i]) < minfrontdistance and float(msg.ranges[i]) != 0.0):
            obstacle = True

        if(float(msg.ranges[i]) <= stopdist):
            stop = 1
            move.linear.x = 0.0
            move.angular.z = 0.0
            pub.publish(move)
        
        if(i < (size / 2)):
            minlid = i
            minleft = min(minleft, float(msg.ranges[i]))
            maxleft = max(maxleft, float(msg.ranges[i]))
        else:
            minrid = i
            minright = min(minright, float(msg.ranges[i]))
            maxright = max(maxright, float(msg.ranges[i]))
            
        for i in range(330, 360):
            if(float(msg.ranges[i]) < minfrontdistance and float(msg.ranges[i]) != 0.0):
                obstacle = True

def rotate():
    global obstacle

    velocity_publisher = rospy.Publisher(nodename + "/cmd_vel", Twist, queue_size=10)
    vel_msg = Twist()

    print("Let's rotate your robot")
    clockwise = True  # input("Clockwise?: ") #True or false

    # Converting from angles to radians
    angular_speed = 20.0 * 2 * 3.14 / 360
    relative_angle = 90.0 * 2 * 3.14 / 360

    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0

    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    velocity_publisher.publish(vel_msg)
    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    r = rospy.Rate(40)
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
    global obstacle

    while obstacle != True:
        move.linear.x = 0.1
        move.angular.z = 0.0
        pub.publish(move)
        rate.sleep()

    rotate()
    controlrobot()


# sub = rospy.Subscriber('/base_scan', LaserScan, callback)
sub = rospy.Subscriber(nodename + "/scan", LaserScan, callback)
controlrobot()
rospy.spin()
