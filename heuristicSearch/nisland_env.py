from node import Node
from env import GridEnvironment

class NIslandGridEnvironment(GridEnvironment):

    def __init__(self, envMap, rows, cols, islandPoints):
        super(NIslandGridEnvironment, self).__init__(envMap, rows, cols)
        self.setIslandNodes(islandPoints)

    def setIslandNodes(self, islandPoints):
        self.islandNodeIds = []
        self.activeIslandNodes = []

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
        if node not in self.activeIslandNodes:
            self.activeIslandNodes.append(node)

    def getIslandThresh(self):
        return self.islandThresh

    def gValue(self, node):
        return node.getG() + node.getH1()

    def hValue(self, node, island=None):
        """Currently island may be just a single node, but this should be extended
        to handle a set of islands."""
        # XXX Ideally, the node should remember its h-value so that it can be used
        # in the Refinement stage. Hence, this should use node.h instead of the
        # heuristic function.
        if island:
            return self.heuristic(node, island) + self.heuristic(island, self.goal)
        else:
            return self.heuristic(node, self.goal)

    def fValue(self, node, island=None):
        return self.gValue(node) + 2*self.hValue(node, island)

    def getChildrenIslandsAndCosts(self, node):
        if(not self.graph.has_key(node.getNodeId())):
            self.graph[node.getNodeId()] = node
        point = self.getPointFromId(node.getNodeId())

        children, edgeCosts = self.getNeighbours(point[0], point[1])
        childrenNodes, dummyChildrenNodes, dummyEdgeCosts = [], [], []
        for child in children:
            nodeId = self.getIdFromPoint(child)

            if(self.graph.has_key(nodeId)):
                childNode = self.graph[nodeId]
                childrenNodes.append(childNode)

            else:
                childNode = Node(nodeId)
                self.graph[nodeId] = childNode
                childrenNodes.append(childNode)

        inactiveIslandNodeIds = set(self.islandNodeIds) - set( map(lambda x:
            x.getNodeId(), self.activeIslandNodes) )
        print (inactiveIslandNodeIds)

        for nodeId in inactiveIslandNodeIds:
            if (self.distanceBetweenNodes(node.getNodeId(), nodeId) <
            self.islandThresh):
                childIslandNode = self.graph[nodeId]
                cost = self.heuristic(node, childIslandNode)
                dummyChildrenNodes.append(childIslandNode)
                dummyEdgeCosts.append(cost)
                self.activateIslandNode( Node( nodeId ) )
                print(cost)

        return (childrenNodes, dummyChildrenNodes, edgeCosts, dummyEdgeCosts)
