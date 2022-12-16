#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt, pi

class TurtlebotAbstract():
    def __init__(self, nodeName):
        self.vel_publisher = rospy.Publisher(f"{nodeName}/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        self.pose_subscriber = rospy.Subscriber(f"{nodeName}/odom", Odometry, self.update_pose)

        self.odom = Odometry()
        self.pose = self.odom.pose.pose
        self.roll = self.pitch = self.yaw = 0.0

    def update_pose(self, data):
        self.pose = data.pose.pose
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        
        pass

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.position.x - self.pose.position.x), 2) + pow((goal_pose.position.y - self.pose.position.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)
        
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.position.y - self.pose.position.y, goal_pose.position.x - self.pose.position.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.yaw)

    def move2goal(self):
        print(f"{self.robotName} started moving to ({self.goalX}, {self.goalY})")

        newOdom = Odometry()
        goal_pose = newOdom.pose.pose
        goal_pose.position.x = self.goalX 
        goal_pose.position.y = self.goalY
        dist_tolerance = 0.15

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= dist_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)
            
            # if facing wrong direction, turn in place
            if(abs(self.angular_vel(goal_pose)) > pi / 2):
                vel_msg.linear.x = 0
                vel_msg.angular.z = self.angular_vel(goal_pose)

            self.vel_publisher.publish(vel_msg)

            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_publisher.publish(vel_msg)

        print(f"{self.robotName} reached its destination!!!")


        