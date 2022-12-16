#!/usr/bin/env python3

import rospy
from TurtleBotFinal2 import TurtlebotTask2
import sys

robotId = int(sys.argv[1])

def readParams():
    try:
        horizontalR = rospy.get_param(f"R_{robotId}")
        goalX = rospy.get_param(f"GoalX_{robotId}")
        goalY = rospy.get_param(f"GoalY_{robotId}")
        return horizontalR, goalX, goalY
    except:
        print("Error reading parameters")
        return None


def main():
    print("Starting Project Task 2")
    rospy.init_node('projecttask2', anonymous=True)

    robot = TurtlebotTask2(robotId - 1)
    horizontalR, goalX, goalY = readParams()
    robot.setGoal(goalX, goalY)
    robot.horizontalParam = horizontalR
    print(robot)


if __name__ == '__main__':
    try:
        main()    
    except rospy.ROSInterruptException:
        pass
