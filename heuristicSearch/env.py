from node import Node 

class Environment(object):
    def __init__(self, envMap):
        self.envMap = envMap

    def isValidPoint(self, point):
        pass

class GridEnvironment(Environment):
    def __init__(self, envMap, rows, cols):
        super(GridEnvironment, self).__init__(envMap)
        self.rows = rows
        self.cols = cols
        self.graph = {}

    def distanceBetweenNodes(self, start, end):
        """Calculates Euclidean distance between two nodes"""
        startPoint, endPoint = map(self.getPointFromId, [start, end])
        return ((startPoint[0] - endPoint[0])**2 + (startPoint[1] -
            endPoint[1])**2)**.5

    def isValidPoint(self, point):
        if self.envMap[point[0], point[1]] < 50:
            return False
        else:
            return True

    def getNeighbours(self, row, col):
        """Returns 8 connected neighbours from the grid. Does validity check f
        the neighbours"""
        neighbours = []
        edgeCosts = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if(not(i == 0 and j == 0)):
                    r = row + i
                    c = col + j
                    if((r >= 0) and (r < self.rows) and (c >= 0) and (c <
                        self.cols)):
                        if self.isValidPoint((r, c)):
                            neighbours.append((r, c))
                            if(i == 0):
                                edgeCosts.append(1)
                            elif j == 0:
                                edgeCosts.append(1)
                            else:
                                edgeCosts.append(2)
        #print(neighbours)
        return (neighbours, edgeCosts)

    def getChildrenAndCosts(self, node):
        if(not self.graph.has_key(node.getNodeId())):
            self.graph[node.getNodeId()] = node
        point = self.getPointFromId(node.getNodeId())

        children, edgeCosts = self.getNeighbours(point[0], point[1])
        childrenNodes = []
        for child in children:
            nodeId = self.getIdFromPoint(child)

            if(self.graph.has_key(nodeId)):
                childNode = self.graph[nodeId]
                childrenNodes.append(childNode)

            else:
                childNode = Node(nodeId)
                self.graph[nodeId] = childNode
                childrenNodes.append(childNode)

        return (childrenNodes, edgeCosts)

    def getIdFromPoint(self, gridPoint):
        return gridPoint[0]*self.cols + gridPoint[1]

    def getPointFromId(self, Id):
        return (Id//self.cols, Id%self.cols)

    def ancestoryContainsNode(self, currNode, nodeToFind, thresh):
        node = currNode.getParent()
        while(node != None and self.distanceBetweenNodes(node.getNodeId(),
            nodeToFind.getNodeId()) < thresh and node != nodeToFind):
            node = node.getParent()
        if node == nodeToFind:
            return True
        else:
            return False

    def addNode(self, newNode):
        self.graph[newNode.getNodeId()] = newNode

    def euclideanHeuristic(self, currNode, goalNode):
        currPoint = self.getPointFromId(currNode.getNodeId())
        goalPoint = self.getPointFromId(goalNode.getNodeId())

        return ((currPoint[0] - goalPoint[0])**2 + (currPoint[1] -
            goalPoint[1])**2)**.5

    def diagonalHeuristic(self, currNode, goalNode):
        currPoint = self.getPointFromId(currNode.getNodeId())
        goalPoint = self.getPointFromId(goalNode.getNodeId())

        dr = abs(currPoint[0] - goalPoint[0])
        dc = abs(currPoint[1] - goalPoint[1])
        D = 1 
        D2 = 1.5
        return D * (dr + dc) + (D2 - 2 * D) * min(dr, dc)

    def setHeuristic(self, heuristicType=1):
        self.heuristicType = heuristicType

    def heuristic(self, currNode, goalNode, *args):
        if self.heuristicType == 0:
            return self.euclideanHeuristic(currNode, goalNode, *args)
        else:
            return self.diagonalHeuristic(currNode, goalNode, *args)

class IslandGridEnvironment(GridEnvironment):
    
    def __init__(self, envMap, rows, cols, islandPoints):
        super(IslandGridEnvironment, self).__init__(envMap, rows, cols)
        self.setIslandNodes(islandPoints)

    def setIslandNodes(self, islandPoints):
        self.islandNodeIds = []
        self.activatedIslandNodes = []

        for point in islandPoints:
           islandId = self.getIdFromPoint(point) 
           self.islandNodeIds.append(islandId)
           if not self.graph.has_key(islandId):
               self.graph[islandId] = Node(islandId)

    def setIslandThresh(self, thresh):
        self.islandThresh = thresh

    def getIslandNodes(self):
        islandNodes = []
        for islandId in self.islandNodeIds:
            islandNodes.append(self.graph[islandId])
        return islandNodes

    def activateIslandNode(self, node):
        if node not in self.activatedIslandNodes:
            self.activatedIslandNodes.append(node)

    def getIslandThresh(self):
        return self.islandThresh

    def getChildrenWithIslandsAndCosts(self, node):
        if(not self.graph.has_key(node.getNodeId())):
            self.graph[node.getNodeId()] = node
        point = self.getPointFromId(node.getNodeId())

        children, edgeCosts = self.getNeighbours(point[0], point[1])
        childrenNodes = []
        for child in children:
            nodeId = self.getIdFromPoint(child)

            if(self.graph.has_key(nodeId)):
                childNode = self.graph[nodeId]
                childrenNodes.append(childNode)

            else:
                childNode = Node(nodeId)
                self.graph[nodeId] = childNode
                childrenNodes.append(childNode)

        flag = 0
        for nodeId in self.islandNodeIds:
            if (self.distanceBetweenNodes(node.getNodeId(), nodeId) <
            self.islandThresh):
                # XXX Assumes that one node is near only one island node.
                childIslandNode = self.graph[nodeId]
                cost = self.heuristic(node, childIslandNode)
                childrenNodes.append(childIslandNode)
                edgeCosts.append(cost)
                flag = 1
                break
        if node.checkDummyG():
            for childNode in childrenNodes:
                childNode.setHasDummyG(True)

        return (childrenNodes, edgeCosts, flag)
