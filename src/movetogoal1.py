#! /usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

nodeid = str(sys.argv[1])
nodename = f"/turtle{nodeid}"

class Turtlebot:
    def __init__(self):
        rospy.init_node('turtletogoal', anonymous=True)
        self.pose_subscriber = rospy.Subscriber(f"{nodename}/pose", Pose, self.update_pose)
        self.vel_publisher = rospy.Publisher(f"{nodename}/cmd_vel", Twist, queue_size=10)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        self.pose = data

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)
        
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self):
        goal_pose = Pose()
        goal_pose.x = float(input("x: "))
        goal_pose.y = float(input("y: "))
        dist_tolerance = float(input("dist_tolerance: "))

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= dist_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.angular.z = self.angular_vel(goal_pose)

            self.vel_publisher.publish(vel_msg)

            self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_publisher.publish(vel_msg)

        # rospy.spin()


if __name__ == "__main__":
    try:
        t = Turtlebot()
        t.move2goal()

    except rospy.ROSInterruptException:
        pass
