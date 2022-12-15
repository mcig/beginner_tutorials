#!/usr/bin/env python3

# rosservice call /reset_positions
import rospy
from std_srvs.srv import Empty

def main():
    rospy.init_node('reset_ros', anonymous=True)
    rospy.wait_for_service('/reset_positions')
    try:
        reset_positions = rospy.ServiceProxy('/reset_positions', Empty)
        reset_positions()
        print("Resetted positions for stageros")
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == '__main__':
    main()