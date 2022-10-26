import random
from utils import Directions

class Cell:
    def __init__(self, id, coords, isObstacle=False, isReward=0, isStart=False):
        self.id = id # format: int
        self.coords = coords # format: (row: int, column: int)
        self.isObstacle = isObstacle
        self.isReward = isReward
        self.isStart = isStart
        self.isVisited = False
        self.exploredFrom = None # Cell
        self.neighbors = [
            None, # up Cell
            None, # right Cell
            None, # down Cell
            None  # left Cell
        ]

    def fillMyNeighbors(self, map, size):
        # up
        if self.coords[0] > 0:
            self.neighbors[0] = map[self.coords[0]-1][self.coords[1]]
        # right
        if self.coords[1] < size-1:
            self.neighbors[1] = map[self.coords[0]][self.coords[1]+1]
        # down
        if self.coords[0] < size-1:
            self.neighbors[2] = map[self.coords[0]+1][self.coords[1]]
        # left
        if self.coords[1] > 0:
            self.neighbors[3] = map[self.coords[0]][self.coords[1]-1]

    def getDirectionToCell(self, otherCell):
        if self.neighbors[0] == otherCell:
            return Directions.NORTH
        elif self.neighbors[1] == otherCell:
            return Directions.EAST
        elif self.neighbors[2] == otherCell:
            return Directions.SOUTH
        elif self.neighbors[3] == otherCell:
            return Directions.WEST
        else:
            return None


class RoboMap:
    def __init__(self, N=4):
        self.roboMap = []
        self.size = N
        self.startCell = None
        
        for i in range(N):
            self.roboMap.append([])
            for j in range(N):
                id = i * N + j # 0 to 15 etc.
                currCell = Cell(id, (i, j))
                self.roboMap[i].append(currCell)
                
        # fill neighbors
        for i in range(N):
            for j in range(N):
                self.roboMap[i][j].fillMyNeighbors(self.roboMap, N)

    def getCell(self, id):
        # as id was calculated as i * N + j, we can get the cell by performing the reverse operation
        row = int(id / self.size)
        column = id % self.size

        return self.roboMap[row][column]

    def pickRandomRewardCells(self, count=3, bannedCellIds=[]):
        # pick 3 random distinct cells
        randRange = list(range(0, self.size**2))
        
        # remove banned cells
        for cellId in bannedCellIds:
            randRange.remove(cellId)
        
        rewardCellIds = random.sample(randRange, count)

        # set the cells as reward cells
        for id in rewardCellIds:
            self.getCell(id).isReward = 1

    def initialize(self):
        startCell = self.getCell(12)
        startCell.isStart = True

        self.startCell = startCell

        obstacleIds = [0, 3, 6]

        for id in obstacleIds:
            self.getCell(id).isObstacle = True
        
        bannedCellIds = [12] + obstacleIds

        self.pickRandomRewardCells(bannedCellIds=bannedCellIds)

    def __str__(self):
        # pretty print as a map
        mapStr = ""
        for i in range(self.size):
            for j in range(self.size):
                if(self.roboMap[i][j].isStart):
                    mapStr += "🤖"
                elif(self.roboMap[i][j].isReward):
                    mapStr += "⭐"
                elif(self.roboMap[i][j].isObstacle):
                    mapStr += "🧱"
                else:
                    mapStr += str(self.roboMap[i][j].id)
                mapStr += " \t"
            mapStr += "\n"

        return mapStr


class DFSAlgorithmRunner:
    def __init__(self, roboMap, rewardDepth):
        self.roboMap = roboMap
        self.rewardDepth = rewardDepth
        self.exploredCells = []
        self.rewardCells = []

    def run(self):
        self.exploreIterative(self.roboMap.startCell)

        # print the explored cells
        print("Explored cells: ")
        for cell in self.exploredCells:
            print(cell.id, end=" ")
        print()

        # print the reward cells
        print("Reward cells: ")
        for cell in self.rewardCells:
            print(cell.id, end=" ")
        print()

        return self.exploredCells

    def explore(self, cell, depth):
        if cell.isVisited:
            return
        
        if len(self.rewardCells) == self.rewardDepth:
            return

        cell.isVisited = True
        self.exploredCells.append(cell)

        if cell.isReward:
            self.rewardCells.append(cell)

        for neighbor in cell.neighbors:
            if neighbor and not neighbor.isObstacle:
                self.explore(neighbor, depth + 1)

        return

    def exploreIterative(self, cell):
        stack = []
        stack.append(cell)
        isBacktracking = False

        while len(stack) > 0:
            currentCell = stack.pop()
            if currentCell.isVisited:
                self.exploredCells.append(currentCell)
                continue

            if len(self.rewardCells) == self.rewardDepth:
                return

            currentCell.isVisited = True
            self.exploredCells.append(currentCell)

            if currentCell.isReward:
                self.rewardCells.append(currentCell)

            for neighbor in reversed(currentCell.neighbors):
                if neighbor and not neighbor.isObstacle:
                    stack.append(neighbor)
        
        return


        # while len(stack) > 0:
        #     currentCell = stack.pop()
        #     if currentCell.isVisited:
        #         continue

        #     if len(self.rewardCells) == self.rewardDepth:
        #         break

        #     currentCell.isVisited = True
        #     self.exploredCells.append(currentCell)

        #     if currentCell.isReward:
        #         self.rewardCells.append(currentCell)

        #     for neighbor in reversed(currentCell.neighbors):
        #         if neighbor and not neighbor.isObstacle:
        #             stack.append(neighbor)