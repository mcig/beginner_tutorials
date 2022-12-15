#! /usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from TurtleBotAbstract import TurtlebotAbstract

class Turtlebot(TurtlebotAbstract):
    def __init__(self, nodeId):
        self.nodeId = nodeId
        self.nodeName = f"robot_{nodeId}"
        super().__init__(self.nodeName)

        self.robotName = f"robot_{nodeId + 1}"
        self.targetNodeName = f"robot_{(nodeId + 1) % 3}"
        self.pub_kickstartNext = rospy.Publisher(f"{self.targetNodeName}/kickstart", Bool, queue_size=10)

        # flags for multirobot control
        self.pause = False
        self.kickstartedNextBot = False

    def check_obstacle(self, msg):
        midPoint = len(msg.ranges) // 2
        
        minus15 = msg.ranges[:midPoint - 15]
        plus15 = msg.ranges[midPoint + 15:]
        
        # if there is an obstacle closer than 0.5 in front of the robot, pause the robot
        if (min(minus15) < 0.5 or min(plus15) < 0.5):
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
    
        