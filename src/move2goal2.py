#! /usr/bin/env python3

import rospy
import sys
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pow, atan2, sqrt

nodeid = str(sys.argv[1])
multipleBots = sys.argv[2]
nodename = f"/robot_{nodeid}"

class Turtlebot:
    def __init__(self):
        rospy.init_node('turtletogoal', anonymous=True)
        self.vel_publisher = rospy.Publisher(f"{nodename}/cmd_vel" if multipleBots else "/cmd_vel", Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber(f"{nodename}/odom" if multipleBots else "/odom", Odometry, self.update_pose)

        self.rate = rospy.Rate(10)

        self.odom = Odometry()
        self.pose = self.odom.pose.pose
        self.roll = self.pitch = self.yaw = 0.0

    def update_pose(self, data):
        self.pose = data.pose.pose
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        
        #print(self.yaw)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.position.x - self.pose.position.x), 2) + pow((goal_pose.position.y - self.pose.position.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)
        
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.position.y - self.pose.position.y, goal_pose.position.x - self.pose.position.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.yaw)

    def move2goal(self):
        newOdom = Odometry()
        goal_pose = newOdom.pose.pose
        goal_pose.position.x = float(input("x: "))
        goal_pose.position.y = float(input("y: "))
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
