#! /usr/bin/env python3

from robomap import DFSAlgorithmRunner, RoboMap
from robot import Robot
from ros_movement_executor import ROSMovementExecutor
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('move_robot_q1', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def main():
    q2_map = RoboMap(4)
    q2_map.initialize()
    print("Map initialized: ")
    print(q2_map)
    robot = Robot(q2_map)
    robotMovementExecutor = ROSMovementExecutor(rospy, pub, robot)
    # dummy go forward for fixing weird bug
    robotMovementExecutor.goForward()
    #
    dfsRunner = DFSAlgorithmRunner(q2_map, 3)
    exploredCells = dfsRunner.run()
    robotMovementExecutor.animateRobot(exploredCells)

    return


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
