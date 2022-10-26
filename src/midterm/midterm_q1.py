#! /usr/bin/env python3

from robomap import DFSAlgorithmRunner, RoboMap
from robot import Robot
from ros_movement_executor import ROSMovementExecutor
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('move_robot_q1', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def main():
    q1_map = RoboMap(4)
    q1_map.initialize()
    print("Map initialized: ")
    print(q1_map)
    robot = Robot(q1_map)
    robotMovementExecutor = ROSMovementExecutor(rospy, pub, robot)
    dfsRunner = DFSAlgorithmRunner(q1_map, 2)
    exploredCells = dfsRunner.run()
    robotMovementExecutor.animateRobot(exploredCells)

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
