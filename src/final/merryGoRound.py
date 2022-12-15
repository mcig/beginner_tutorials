#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from TurtleBotFinal import Turtlebot
from std_msgs.msg import Bool

def readOdomOfRobot(robot):
    odom = rospy.wait_for_message(f"{robot.nodeName}/odom", Odometry, timeout=5)
    return odom

def initiatesRobots(roboCount):
    robots = []
    for i in range(roboCount):
        robot = Turtlebot(i)
        robots.append(robot)

    return robots

def startMovement(robot):
    robot.move2goal()
    
def main():
    rospy.init_node('merry_go_round', anonymous=True)

    (robot1, robot2, robot3) = initiatesRobots(3)

    robot1.setGoal(readOdomOfRobot(robot2))
    print(robot1)
    robot2.setGoal(readOdomOfRobot(robot3))
    print(robot2)
    robot3.setGoal(readOdomOfRobot(robot1))
    print(robot3)

    # parallelize the movement of the robots
    rospy.Subscriber(f"{robot2.nodeName}/kickstart", Bool, lambda _:startMovement(robot2))
    rospy.Subscriber(f"{robot3.nodeName}/kickstart", Bool, lambda _:startMovement(robot3))

    startMovement(robot1)

    rospy.spin()
    
    return




if __name__ == '__main__':
    try:
        main()    
    except rospy.ROSInterruptException:
        pass
