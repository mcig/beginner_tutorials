from utils import Directions

class Robot:
    def __init__(self, map):
        self.map = map
        self.currentCell = map.startCell
        self.visitedCells = [self.currentCell]
        self.facingDirection = Directions.EAST
        