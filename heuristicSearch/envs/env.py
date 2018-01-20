from ..graph.node import Node

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

    def goal(self, node=None):
        if node:
            self.goal = node
        else:
            return self.goal

    def squaredDistanceBetween( self, start, end ):
        startPoint, endPoint = map(self.getPointFromId, [start, end])
        return ((startPoint[0] - endPoint[0])**2 + (startPoint[1] -
            endPoint[1])**2)

    def distanceBetweenNodes(self, start, end):
        """Calculates Euclidean distance between two nodes"""
        return self.squaredDistanceBetween( start, end )**.5

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

    def addNode(self, nodeId):
        if not self.graph.has_key(nodeId):
            self.graph[nodeId] = Node(nodeId)

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

    def setHeuristicType(self, heuristicType=1):
        self.heuristicType = heuristicType

    def setHeuristic(self, heuristic):
        """heuristic takes in currNode and goalNode and returns the heuristic
        cost."""
        self.heuristic = heuristic

    def heuristic(self, currNode, goalNode, *args):
        if self.heuristicType == 0:
            return self.euclideanHeuristic(currNode, goalNode, *args)
        else:
            return self.diagonalHeuristic(currNode, goalNode, *args)

    def h(self, node):
        return heuristic(node, goal)
