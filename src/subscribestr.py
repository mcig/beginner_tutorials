#!/usr/bin/env python3
#chmod u+x ./beginner_tutorials/src/subscribestr.py
import rospy
from std_msgs.msg import String

def callback(msg):
    print(f"Received: {msg}")

rospy.init_node('string_sub')

sub = rospy.Subscriber('/words', String, callback)

rospy.spin()

