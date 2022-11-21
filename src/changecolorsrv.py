#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty
from turtlesim.srv import *

rospy.init_node('setcolornode', anonymous=True)
print("1")

color_r = rospy.get_param('r')
color_g = rospy.get_param('g')
color_b = rospy.get_param('b')

print("r: ", color_r)
print("g: ", color_g)
print("b: ", color_b)

print("imdat")
rospy.wait_for_service('clear')
print("2")

rospy.set_param('/turtlesim/background_r', color_r)
rospy.set_param('/turtlesim/background_g', color_g)
rospy.set_param('/turtlesim/background_b', color_b)

clear_background = rospy.ServiceProxy('clear', Empty)
clear_background()

spawnTurtle = rospy.ServiceProxy('spawn', Spawn)
spawnTurtle(2, 2, 0, 'turtle2')

