#! /usr/bin/env python3

from dfs_algorithm_runner import DFSAlgorithmRunner
from robomap import RoboMap
from robot import Robot

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('move_robot_q1', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def main():
    q1_map = RoboMap(4)
    q1_map.initialize()
    print("Map initialized: ")
    print(q1_map)

    robot = Robot(rospy, pub, q1_map)

    dfsRunner = DFSAlgorithmRunner(robot)
    dfsRunner.run(rewardDepth=2)

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
