from utils import Directions

class Robot:
    def __init__(self, rospy, rosPublisher, robomap):
        self.myRos = rospy
        self.myPublisher = rosPublisher
        self.myMap = robomap

        self.currentCell = robomap.startCell
        self.facingDirection = Directions.EAST
        