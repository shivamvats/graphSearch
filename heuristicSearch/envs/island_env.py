from heuristicSearch.graph.node import Node
from env import GridEnvironment

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
