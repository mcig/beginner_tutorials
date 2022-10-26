#!/usr/bin/env python3
#chmod u+x ./beginner_tutorials/src/publishstr.py
import rospy
from std_msgs.msg import String

rospy.init_node('string_pub')
pub = rospy.Publisher('/words', String, queue_size=10)

msg = String("slm knk")
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    print(f"Published: {msg}")
    pub.publish(msg)
    rate.sleep()
    
