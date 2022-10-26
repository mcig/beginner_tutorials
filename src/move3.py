#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import Twist

nodeid = str(sys.argv[1])
# nodename = f"/turtle{nodeid}"
nodename = f"/robot_{nodeid}"
vel = float(f"0.{sys.argv[2]}")

def moveTask():
    rospy.init_node('move_node', anonymous=True)
    pub = rospy.Publisher(f'{nodename}/cmd_vel', Twist, queue_size=10)
    
    vel_msg = Twist()
    vel_msg.linear.x = 0.5
    
    if(nodeid == '0'):
        vel_msg.angular.z = vel or 0.5 

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        moveTask()    
    except rospy.ROSInterruptException:
        pass
