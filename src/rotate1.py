#!/usr/bin/env python3
import math
import sys
import rospy
from geometry_msgs.msg import Twist

nodeid = str(sys.argv[1])
nodename = f"/turtle{nodeid}"
rospy.init_node('rotate_node', anonymous=True)
pub = rospy.Publisher(f'{nodename}/cmd_vel', Twist, queue_size=10)

def rotateTask(speed, angle, clockwise, lspeed = 0.0):
    vel_msg = Twist()

    angular_speed = speed * (math.pi / 180)
    vel_msg.angular.z = -clockwise * abs(angular_speed)
    rate = rospy.Rate(10)
    t0 = rospy.Time.now().to_sec()
    current_angle = 0
    relative_angle = angle * (math.pi / 180)

    while current_angle < relative_angle:
        pub.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed * (t1 - t0)
        rate.sleep()

    vel_msg.angular.z = 0
    pub.publish(vel_msg)


def moveTask(speed, distance, isForward, lspeed = 0.0):
    vel_msg = Twist()

    vel_msg.linear.x = speed if isForward else -abs(speed)

    t0 = rospy.Time.now().to_sec()
    
    rate = rospy.Rate(10)

    current_distance = 0
    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = speed * (t1-t0)
        if current_distance > distance:
            break
        rate.sleep()

    vel_msg.linear.x = 0
    pub.publish(vel_msg)


if __name__ == '__main__':
    try:
        rotateTask(10, 90, -1)
        moveTask(0.5)
        rotateTask(10, 90, 1)
        moveTask(0.5)
        rotateTask(10, 90, 1)
        moveTask(0.5)
    except rospy.ROSInterruptException:
        pass

