from utils import Directions

class Robot:
    def __init__(self, robomap):
        self.myMap = robomap
        self.currentCell = robomap.startCell
        self.facingDirection = Directions.EAST
        