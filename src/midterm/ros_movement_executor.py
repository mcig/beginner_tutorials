import math
from geometry_msgs.msg import Twist

class ROSMovementExecutor:
    def __init__(self, rospy, pub, robot):
        self.rospy = rospy
        self.pub = pub
        self.robot = robot
        self.rate = self.rospy.Rate(10)

    def rotateTask(self, clockwise):
        speed = 20
        angle = 90
        
        vel_msg = Twist()
        angular_speed = speed * (math.pi / 180)
        vel_msg.angular.z = -clockwise * abs(angular_speed)
        t0 = self.rospy.Time.now().to_sec()
        current_angle = 0
        relative_angle = angle * (math.pi / 180)

        while current_angle < relative_angle:
            self.pub.publish(vel_msg)
            t1 = self.rospy.Time.now().to_sec()
            current_angle = angular_speed * (t1 - t0)
            self.rate.sleep()

        vel_msg.angular.z = 0
        self.pub.publish(vel_msg)


    def moveTask(self, distance):
        speed = 0.7
        
        vel_msg = Twist()
        
        vel_msg.linear.x = speed

        t0 = self.rospy.Time.now().to_sec()
        
        current_distance = 0
        while current_distance <= distance:
            self.pub.publish(vel_msg)
            t1 = self.rospy.Time.now().to_sec()
            current_distance = speed * (t1-t0)
            self.rate.sleep()

        vel_msg.linear.x = 0
        self.pub.publish(vel_msg)

    def goRight(self):
        self.rotateTask(1)
        self.moveTask(4)

    def goLeft(self):
        self.rotateTask(-1)
        self.moveTask(4)

    def goForward(self):
        self.moveTask(4)

    def goBackward(self):
        self.rotateTask(1)
        self.rotateTask(1)
        self.moveTask(4)

    # up down 
    def decideRobotMovement(self, toDirection):
        fromDirection = self.robot.facingDirection
        commandToExecute = None

        diffInNumbers = (int(toDirection) - int(fromDirection)) % 4

        if diffInNumbers == 0:
            commandToExecute = self.goForward
        elif diffInNumbers == 1:
            commandToExecute = self.goRight
        elif diffInNumbers == 2:
            commandToExecute = self.goBackward
        else:
            commandToExecute = self.goLeft

        return (commandToExecute, toDirection)

    def moveRobot(self, toDirection):
        (decidedCommand, newFacedDirection) = self.decideRobotMovement(toDirection)
        decidedCommand()
        self.robot.facingDirection = newFacedDirection

    def animateRobot(self, exploredCells):
        exploredCells = exploredCells[1:]
        for nextCell in exploredCells:
            fromCell = self.robot.currentCell
            
            toDirection = fromCell.getDirectionToCell(nextCell)
            self.moveRobot(toDirection)
            
            self.robot.currentCell = nextCell
       
