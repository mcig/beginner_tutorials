#!/usr/bin/env python3

import rospy
from TurtleBotFinal2 import TurtlebotTask2
import sys

robotId = int(sys.argv[1])

def readParams():
    try:
        horizontalR = rospy.get_param(f"R_{robotId}")
        maxSweepCount = rospy.get_param(f"MaxSweepCount_{robotId}")
        firstRotation = rospy.get_param(f"Rotate_{robotId}")
        return horizontalR, maxSweepCount, firstRotation
    except:
        print("Error reading parameters")
        return None


def main():
    print("Starting Project Task 2")
    rospy.init_node(f'projecttask2_{robotId}', anonymous=True)

    robot = TurtlebotTask2(robotId - 1)
    horizontalR, maxSweepCount, firstRotation = readParams()
    robot.horizontalParam = horizontalR
    robot.maxSweepCount = maxSweepCount
    robot.nextRotation = firstRotation
    print(robot)
    
    robot.move_and_sweep()

if __name__ == '__main__':
    try:
        main()    
    except rospy.ROSInterruptException:
        pass
