#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def moveTask():
    rospy.init_node('move_node', anonymous=True)
    pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
    
    vel_msg = Twist()
    vel_msg.angular.z = 1000

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        moveTask()    
    except rospy.ROSInterruptException:
        pass
