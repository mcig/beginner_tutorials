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
        self.nextRotation = 1 # 1 south -1 north
        self.pose = Pose2D()
        self.theta = 0
        self.maxSweepCount = None
        self.pauseForSweep = False

    def check_obstacle_or_move(self, msg):
        midPoint = len(msg.ranges) // 2
        
        minus15 = msg.ranges[midPoint - 15 : midPoint]
        plus15 = msg.ranges[midPoint : midPoint + 15]
        threshold = 1
        
        # if there is an obstacle closer than 0.5 in front of the robot, pause the robot
        if (min(minus15) < threshold or min(plus15) < threshold):
            # encountered an obstacle rotate to related dir
            self.pauseForSweep = True
        
        return

    def sweepEdgeMovement(self):
        self.cool_rotate_to(0 if self.robotName == "robot_1" else 180)

        # move forward R meters
        self.movedist(self.horizontalParam)

        self.cool_rotate_to(270 if self.nextRotation == 1 else 90)

        self.nextRotation *= -1

        pass

    def update_pose(self, msg):
        pose = msg.pose.pose
        self.pose = Pose2D()
        self.pose.x = pose.position.x
        self.pose.y = pose.position.y

        q = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )
        
        m = tf.transformations.quaternion_matrix(q)
        _roll, _pitch, yaw = tf.transformations.euler_from_matrix(m)
        self.theta = yaw

    def move_and_sweep(self):
        rate = rospy.Rate(10)
        msg = Twist()
        speed = 0.5

        while not rospy.is_shutdown():
            msg.linear.x = speed

            self.vel_publisher.publish(msg)
            rospy.wait_for_message(f"{self.nodeName}/odom", Odometry)

            if(self.pauseForSweep):
                if(self.maxSweepCount == 0):
                    print(f"{self.robotName} reached the goal")
                    break

                self.sweepEdgeMovement()
                self.pauseForSweep = False
                self.maxSweepCount -= 1
                
            rate.sleep()

        msg.linear.x = 0
        msg.angular.z = 0
        self.vel_publisher.publish(msg)

    def movedist(self, dist):
        rate = rospy.Rate(10)
        msg = Twist()
        current_distance = 0
        speed = 0.5
        t0 = rospy.Time.now().to_sec()

        while current_distance <= dist:
            msg.linear.x = speed
            t1 = rospy.Time.now().to_sec()
            current_distance = speed * (t1 - t0)

            self.vel_publisher.publish(msg)
            rospy.wait_for_message(f"{self.nodeName}/odom", Odometry)
            rate.sleep()

        msg.linear.x = 0
        msg.angular.z = 0
        self.vel_publisher.publish(msg)

    def cool_rotate_to(self, degrees):
        msg = Twist()
        c = 0.5
        start_theta = self.theta
        targetrad = degrees * pi / 180.0 # convert to radians
        
        if(targetrad == (pi + pi/2)):
            if(self.nextRotation == 1):
                targetrad = -pi/2
            else:
                targetrad = pi/2

            
        while not rospy.is_shutdown():
            msg.linear.x = 0
            msg.angular.z = c * (targetrad - self.theta)
            
            if targetrad - start_theta >= 0 and msg.angular.z <= 0.008:
                break
            elif targetrad - start_theta <= 0 and msg.angular.z >= 0.008:
                break

            if abs(targetrad - self.theta) < 0.01:
                break

            self.vel_publisher.publish(msg)
            rospy.wait_for_message(f"{self.nodeName}/odom", Odometry)
            self.rate.sleep()

        msg.linear.x = 0
        msg.angular.z = 0
        self.vel_publisher.publish(msg)
        print(f"{self.robotName}: Rotated to {degrees}{chr(176)}")

    def __str__(self):
        return f"I am {self.robotName}; my R is {self.horizontalParam} and i will stop after {self.maxSweepCount} sweeps"
    