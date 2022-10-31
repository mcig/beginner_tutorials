from ros_movement_executor import ROSMovementExecutor

class DFSAlgorithmRunner:
    def __init__(self, robot):
        self.robot = robot
        self.roboMap = robot.myMap
        self.movementExecutor = ROSMovementExecutor(robot)
        self.exploredCells = []
        self.rewardCells = []

    def run(self, rewardDepth):
        self.rewardDepth = rewardDepth
        self.exploreIterative(self.roboMap.startCell)

        # print the explored cells
        print("Explored path was: ")
        for cell in self.exploredCells:
            print(cell.id, end=" ")
        print()

        # print the reward cells
        print("Reward cells were: ")
        for cell in self.rewardCells:
            print(cell.id, end=" ")
        print()

        return self.exploredCells

    def exploreIterative(self, cell):
        stack = []
        stack.append(cell)

        while len(stack) > 0:
            if len(self.rewardCells) == self.rewardDepth:
                return
            
            poppedCell = stack.pop()
            
            # backtracking
            if poppedCell.isVisited:
                # special case for id == 1
                if(poppedCell.hasUnvisitedNeighbors() or poppedCell.id == 1):
                    self.exploredCells.append(poppedCell)
                    self.movementExecutor.moveRobotTo(poppedCell)
                continue

            poppedCell.isVisited = True

            # do not push the first cell
            if(poppedCell.id is not cell.id):
                self.exploredCells.append(poppedCell)
                self.movementExecutor.moveRobotTo(poppedCell)

            if poppedCell.isReward:
                self.rewardCells.append(poppedCell)
                print(f"Robot found a reward cell: {poppedCell.id} ⭐")

            for neighbor in reversed(poppedCell.neighbors):
                if(neighbor and not neighbor.isObstacle):
                    stack.append(neighbor)
        
        return
