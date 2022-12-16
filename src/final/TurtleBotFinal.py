#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from TurtleBotAbstract import TurtlebotAbstract
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi
from sensor_msgs.msg import LaserScan

class Turtlebot(TurtlebotAbstract):
    def __init__(self, nodeId):
        self.nodeId = nodeId
        self.nodeName = f"robot_{nodeId}"
        super().__init__(self.nodeName)

        self.robotName = f"robot_{nodeId + 1}"
        self.scanner_subscriber = rospy.Subscriber(f"{self.nodeName}/base_scan", LaserScan, self.check_obstacle)

        self.targetNodeName = f"robot_{(nodeId + 1) % 3}"
        self.pub_kickstartNext = rospy.Publisher(f"{self.targetNodeName}/kickstart", Bool, queue_size=10)

        # flags for multirobot control
        self.pause = False
        self.kickstartedNextBot = False

    def check_obstacle(self, msg):
        midPoint = len(msg.ranges) // 2
        
        minus15 = msg.ranges[midPoint - 15:midPoint]
        plus15 = msg.ranges[midPoint:midPoint + 15]
        threshold = 1.2
        
        # if there is an obstacle closer than 0.5 in front of the robot, pause the robot
        if (min(minus15) < threshold or min(plus15) < threshold):
            self.pause = True
            
            if(not self.kickstartedNextBot):
                self.pub_kickstartNext.publish(True)
                self.kickstartedNextBot = True
        else:
            self.pause = False

        pass

    def setGoal(self, odomData):
        self.goalX = odomData.pose.pose.position.x
        self.goalY = odomData.pose.pose.position.y
        pass

    def __str__(self):
        return f"I am {self.robotName} and my goal is ({self.goalX}, {self.goalY})"
    
    def move2goal(self):
        print(f"{self.robotName} started moving to ({self.goalX}, {self.goalY})")

        newOdom = Odometry()
        goal_pose = newOdom.pose.pose
        goal_pose.position.x = self.goalX 
        goal_pose.position.y = self.goalY
        dist_tolerance = 0.15

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= dist_tolerance:
            if(self.pause):
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.vel_publisher.publish(vel_msg)
                continue
                
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
