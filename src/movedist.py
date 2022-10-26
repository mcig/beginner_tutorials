#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import Twist

nodeid = str(sys.argv[1])
nodename = f"/turtle{nodeid}"

def moveTask():
    rospy.init_node('move_node', anonymous=True)
    #pub = rospy.Publisher(f'{nodename}/cmd_vel', Twist, queue_size=10)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    vel_msg = Twist()
    speed = 0.5
    isForward = True
    distance = 3

    vel_msg.linear.x = speed if isForward else -abs(speed)

    t0 = rospy.Time.now().to_sec()
    
    rate = rospy.Rate(10)

    current_distance = 0
    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_distance = speed * (t1-t0)

        if(current_distance > distance):
            isForward = not isForward 
            vel_msg.linear.x = speed if isForward else -abs(speed)
            current_distance = 0
        rate.sleep()

    vel_msg.linear.x = 0
    pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        moveTask()
    except rospy.ROSInterruptException:
        pass
