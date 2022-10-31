#! /usr/bin/env python3

from dfs_algorithm_runner import DFSAlgorithmRunner
from robomap import RoboMap
from robot import Robot

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('move_robot_q1', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def main():
    q2_map = RoboMap(4)
    q2_map.initialize()
    print("Map initialized: ")
    print(q2_map)

    robot = Robot(rospy, pub, q2_map)

    dfsRunner = DFSAlgorithmRunner(robot)
    dfsRunner.run(rewardDepth=3)

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
