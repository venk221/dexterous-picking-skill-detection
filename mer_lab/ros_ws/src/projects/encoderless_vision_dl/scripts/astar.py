#!/usr/bin/env python3

from heapq import heappush, heappop

class MyAstar(object):
    # init function
    def __init__(self, start, goal, rows, cols, skeleton):
        self.start = start
        self.goal = goal
        self.numRows = rows
        self.numCols = cols
        self.stepSize = 1
        self.clearance = 0
        self.radius = 0
        self.map = skeleton
        self.graph = {}
        self.distance = {}
        self.path = {}
        self.costToCome = {}
        self.costToGo = {}
        self.visited = {}
        
        for row in range(1, self.numRows + 1):
            for col in range(1, self.numCols + 1):
                self.visited[(row, col)] = False
                self.path[(row, col)] = -1
                # self.graph[(row, col)] = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]
                self.graph[(row, col)] = [1, 1, 1, 1, 1, 1, 1, 1]
                self.costToCome[(row, col)] = float('inf')
                self.costToGo[(row, col)] = float('inf')
                self.distance[(row, col)] = float('inf')

                # self.graph[(row, col)] = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]
                self.graph[(row, col)] = [1, 1, 1, 1, 1, 1, 1, 1]
                self.costToCome[(row, col)] = float('inf')
                self.costToGo[(row, col)] = float('inf')
                self.distance[(row, col)] = float('inf')



    # move is within the image dimensions 
    def IsValid(self, currRow, currCol):
        return (currRow >= (1 + self.radius + self.clearance) and currRow <= (self.numRows - self.radius - self.clearance) and currCol >= (1 + self.radius + self.clearance) and currCol <= (self.numCols - self.radius - self.clearance))
   

    # Is the pixel part of skeleton?
    def IsObstacle(self, col, row):
        if self.map[row, col] > 0:
            return False
        return True

    # action move left
    def ActionMoveLeft(self, currRow, currCol):
        if(self.IsValid(currRow, currCol - self.stepSize) and self.IsObstacle(currRow, currCol - self.stepSize) == False and self.visited[(currRow, currCol - self.stepSize)] == False):
            return True
        return False

    # action move right
    def ActionMoveRight(self, currRow, currCol):
        if(self.IsValid(currRow, currCol + self.stepSize) and self.IsObstacle(currRow, currCol + self.stepSize) == False and self.visited[(currRow, currCol + self.stepSize)] == False):
            return True
        return False


    # action move up
    def ActionMoveUp(self, currRow, currCol):
        if(self.IsValid(currRow - self.stepSize, currCol) and self.IsObstacle(currRow - self.stepSize, currCol) == False and self.visited[(currRow - self.stepSize, currCol)] == False):
            return True
        return False


    # action move down
    def ActionMoveDown(self, currRow, currCol):
        if(self.IsValid(currRow + self.stepSize, currCol) and self.IsObstacle(currRow + self.stepSize, currCol) == False and self.visited[(currRow + self.stepSize, currCol)] == False):
            return True
        return False


    # action move right up
    def ActionMoveRightUp(self, currRow, currCol):
        if(self.IsValid(currRow - self.stepSize, currCol + self.stepSize) and self.IsObstacle(currRow - self.stepSize, currCol + self.stepSize) == False and self.visited[(currRow - self.stepSize, currCol + self.stepSize)] == False):
            return True
        return False


    # action move right down
    def ActionMoveRightDown(self, currRow, currCol):
        if(self.IsValid(currRow + self.stepSize, currCol + self.stepSize) and self.IsObstacle(currRow + self.stepSize, currCol + self.stepSize) == False and self.visited[(currRow + self.stepSize, currCol + self.stepSize)] == False):
            return True
        return False


    # action move left down
    def ActionMoveLeftDown(self, currRow, currCol):
        if(self.IsValid(currRow + self.stepSize, currCol - self.stepSize) and self.IsObstacle(currRow + self.stepSize, currCol - self.stepSize) == False and self.visited[(currRow + self.stepSize, currCol - self.stepSize)] == False):
            return True
        return False


    # action move left up
    def ActionMoveLeftUp(self, currRow, currCol):
        if(self.IsValid(currRow - self.stepSize, currCol - self.stepSize) and self.IsObstacle(currRow - self.stepSize, currCol - self.stepSize) == False and self.visited[(currRow - self.stepSize, currCol - self.stepSize)] == False):
            return True
        return False


    # update action
    def UpdateAction(self, currentNode, weight, newRow, newCol):
        new_cost_to_come = self.costToCome[currentNode] + weight
        new_cost_to_go = self.euc_heuristic(newRow, newCol)
        new_distance = new_cost_to_come + new_cost_to_go


        if(self.distance[(newRow, newCol)] > new_distance):
            self.distance[(newRow, newCol)] = new_distance
            self.costToCome[(newRow, newCol)] = new_cost_to_come
            self.costToGo[(newRow, newCol)] = new_cost_to_go
            self.path[(newRow, newCol)] = currentNode
            return True
        return False


    def euc_heuristic(self, row, col):
        return (((self.goal[0] - row)**2) + ((self.goal[1] - col) **2))


    # A-star
    def search(self):
        # mark source node and create queue
        exploredStates = []
        queue = []
        self.costToCome[self.start] = 0
        self.costToGo[self.start] = self.euc_heuristic(self.start[0], self.start[1])
        self.distance[self.start] = self.costToCome[self.start] + self.costToGo[self.start]
        heappush(queue, (self.distance[self.start], self.costToCome[self.start], self.start))

        while len(queue) > 0:
            # get current node
            _,_,currentNode = heappop(queue)
            self.visited[currentNode] = True
            exploredStates.append(currentNode)

            # if goal reached then break
            if currentNode[0] == self.goal[0] and currentNode[1] == self.goal[1]:
                break

            # traverse connected neighbors
            if(self.ActionMoveLeft(currentNode[0], currentNode[1])):
                updateHeap = self.UpdateAction(currentNode, self.graph[currentNode][0], currentNode[0], currentNode[1] - self.stepSize)
                if(updateHeap):
                    heappush(queue, (self.distance[(currentNode[0], currentNode[1] - self.stepSize)], self.costToCome[(currentNode[0], currentNode[1] - self.stepSize)], (currentNode[0], currentNode[1] - self.stepSize)))
            
            if(self.ActionMoveRight(currentNode[0], currentNode[1])):
                updateHeap = self.UpdateAction(currentNode, self.graph[currentNode][1], currentNode[0], currentNode[1] + self.stepSize)
                if(updateHeap):
                    heappush(queue, (self.distance[(currentNode[0], currentNode[1] + self.stepSize)], self.costToCome[(currentNode[0], currentNode[1] + self.stepSize)], (currentNode[0], currentNode[1] + self.stepSize)))
                    
            if(self.ActionMoveUp(currentNode[0], currentNode[1])):
                updateHeap = self.UpdateAction(currentNode, self.graph[currentNode][2], currentNode[0] - self.stepSize, currentNode[1])
                if(updateHeap):
                    heappush(queue, (self.distance[(currentNode[0] - self.stepSize, currentNode[1])], self.costToCome[(currentNode[0] - self.stepSize, currentNode[1])], (currentNode[0] - self.stepSize, currentNode[1])))
                    
            if(self.ActionMoveDown(currentNode[0], currentNode[1])):
                updateHeap = self.UpdateAction(currentNode, self.graph[currentNode][3], currentNode[0] + self.stepSize, currentNode[1])
                if(updateHeap):
                    heappush(queue, (self.distance[(currentNode[0] + self.stepSize, currentNode[1])], self.costToCome[(currentNode[0] + self.stepSize, currentNode[1])], (currentNode[0] + self.stepSize, currentNode[1])))
                    
            if(self.ActionMoveRightDown(currentNode[0], currentNode[1])):
                updateHeap = self.UpdateAction(currentNode, self.graph[currentNode][4], currentNode[0] + self.stepSize, currentNode[1] + self.stepSize)
                if(updateHeap):
                    heappush(queue, (self.distance[(currentNode[0] + self.stepSize, currentNode[1] + self.stepSize)], self.costToCome[(currentNode[0] + self.stepSize, currentNode[1] + self.stepSize)], (currentNode[0] + self.stepSize, currentNode[1] + self.stepSize)))
                    
            if(self.ActionMoveRightUp(currentNode[0], currentNode[1])):
                updateHeap = self.UpdateAction(currentNode, self.graph[currentNode][5], currentNode[0] - self.stepSize, currentNode[1] + self.stepSize)
                if(updateHeap):
                    heappush(queue, (self.distance[(currentNode[0] - self.stepSize, currentNode[1] + self.stepSize)], self.costToCome[(currentNode[0] - self.stepSize, currentNode[1] + self.stepSize)], (currentNode[0] - self.stepSize, currentNode[1] + self.stepSize)))
                    
            if(self.ActionMoveLeftUp(currentNode[0], currentNode[1])):
                updateHeap = self.UpdateAction(currentNode, self.graph[currentNode][6], currentNode[0] - self.stepSize, currentNode[1] - self.stepSize)
                if(updateHeap):
                    heappush(queue, (self.distance[(currentNode[0] - self.stepSize, currentNode[1] - self.stepSize)], self.costToCome[(currentNode[0] - self.stepSize, currentNode[1] - self.stepSize)], (currentNode[0] - self.stepSize, currentNode[1] - self.stepSize)))
                    
            if(self.ActionMoveLeftDown(currentNode[0], currentNode[1])):
                updateHeap = self.UpdateAction(currentNode, self.graph[currentNode][7], currentNode[0] + self.stepSize, currentNode[1] - self.stepSize)
                if(updateHeap):
                    heappush(queue, (self.distance[(currentNode[0] + self.stepSize, currentNode[1] - self.stepSize)], self.costToCome[(currentNode[0] + self.stepSize, currentNode[1] - self.stepSize)], (currentNode[0] + self.stepSize, currentNode[1] - self.stepSize)))

            
        # return if no optimal path
        if(self.distance[self.goal] == float('inf')):
            return (exploredStates, [], self.distance[self.goal])

        # backtrack path
        backtrackStates = []
        node = self.goal
        while(self.path[node] != -1):
            backtrackStates.append(node)
            node = self.path[node]
        backtrackStates.append(self.start)
        backtrackStates = list(reversed(backtrackStates))      
        return (exploredStates, backtrackStates, self.distance[self.goal])