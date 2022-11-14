#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyResponse
from turtlesim.srv import *

rospy.init_node('setcolornode', anonymous=True)
color_r = 120#rospy.get_param('r')
color_g = 60#rospy.get_param('g')
color_b = 50#rospy.get_param('b')

print("r: ", color_r)
print("g: ", color_g)
print("b: ", color_b)

rospy.wait_for_service('clear')

rospy.set_param('/turtlesim/background_r', color_r)
rospy.set_param('/turtlesim/background_g', color_g)
rospy.set_param('/turtlesim/background_b', color_b)

clear_background = rospy.ServiceProxy('clear', Empty)
clear_background()


