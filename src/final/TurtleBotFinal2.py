#! /usr/bin/env python3

import rospy
from TurtleBotAbstract import TurtlebotAbstract
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from math import pi
from sensor_msgs.msg import LaserScan
import tf.transformations 

class TurtlebotTask2(TurtlebotAbstract):
    def __init__(self, nodeId):
        self.nodeId = nodeId
        self.nodeName = f"robot_{nodeId}"
        super().__init__(self.nodeName)

        self.robotName = f"robot_{nodeId + 1}"
        self.scanner_subscriber = rospy.Subscriber(f"{self.nodeName}/base_scan", LaserScan, self.check_obstacle_or_move)

        self.horizontalParam = None
        self.nextRotation = 1 # 1 = right, -1 = left
        self.pose = Pose2D()
        self.theta = 0

    def check_obstacle_or_move(self, msg):
        midPoint = len(msg.ranges) // 2
        
        minus15 = msg.ranges[midPoint - 15:midPoint]
        plus15 = msg.ranges[midPoint:midPoint + 15]
        threshold = 1
        
        # if there is an obstacle closer than 0.5 in front of the robot, pause the robot
        if (min(minus15) < threshold or min(plus15) < threshold):
            # encountered an obstacle rotate to related dir
            print("Encountered an edge, rotating")
            self.sweepEdgeMovement()
        else:
            # no obstacle, move forward
            print("No obstacle, moving forward")
            self.move(threshold)
        pass

    def sweepEdgeMovement(self):
        self.cool_rotate(90 * self.nextRotation)

        # move forward R meters
        self.move(self.horizontalParam)

        self.cool_rotate(90 * self.nextRotation)

        self.nextRotation *= -1

        pass

    def update_pose(self, msg):
        pose = msg.pose.pose
        self.pose = Pose2D()
        self.pose.x = pose.position.x
        self.pose.y = pose.position.y

        q = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        
        m = tf.transformations.quaternion_matrix(q)
        _roll, _pitch, yaw = tf.transformations.euler_from_matrix(m, 'rzyx')
        self.theta = yaw

    def setGoal(self, x,y):
        self.goalX = x
        self.goalY = y
        pass

    def move(self, dist):
        rate = rospy.Rate(10)
        msg = Twist()

        while not rospy.is_shutdown() and self.pose.x < dist:
            msg.linear.x = 0.5
            self.vel_publisher.publish(msg)
            rospy.wait_for_message(f"{self.nodeName}/odom", Odometry)
            rate.sleep()

        msg.linear.x = 0
        msg.angular.z = 0
        self.vel_publisher.publish(msg)
        print(f"Moved {dist}m forward")

    def cool_rotate(self, degrees):
        msg = Twist()

        msg.linear.x = 0
        msg.angular.z = 0.2

        targetrad = degrees * pi / 180.0
        start_theta = self.theta
        c = 0.5

        while not rospy.is_shutdown():
            print("Target: ", msg.angular.z)
            msg.linear.x = 0
            msg.angular.z = c * (targetrad - self.theta)

            if targetrad - start_theta > 0 and msg.angular.z < 0.008:
                break
            elif targetrad - start_theta < 0 and msg.angular.z > 0.008:
                break

            self.vel_publisher.publish(msg)
            rospy.wait_for_message(f"{self.nodeName}/odom", Odometry)
            self.rate.sleep()

        msg.linear.x = 0
        msg.angular.z = 0
        self.vel_publisher.publish(msg)
        print(f"Rotated {degrees} degrees")

    def __str__(self):
        return f"I am {self.robotName} and my R is {self.horizontalParam} and my goal is ({self.goalX}, {self.goalY})"
    