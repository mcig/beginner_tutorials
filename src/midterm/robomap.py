import random

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
        startCell.isVisited = True

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


