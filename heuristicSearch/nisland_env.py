from node import Node
from env import GridEnvironment

class NIslandGridEnvironment(GridEnvironment):

    def __init__(self, envMap, rows, cols, islandPoints):
        super(NIslandGridEnvironment, self).__init__(envMap, rows, cols)
        self.setIslandNodes(islandPoints)
        self.inactiveIslandNodeIds = []

    def setIslandNodes(self, islandPoints):
        self.islandNodeIds = []
        # Dummy edge added.
        self.activeIslandNodes = []
        # Island nodes used yet.
        self.inactiveIslandNodes = []

        for point in islandPoints:
            islandId = self.getIdFromPoint(point)
            self.islandNodeIds.append(islandId)
            if not self.graph.has_key(islandId):
                self.graph[islandId] = Node(islandId)
            self.inactiveIslandNodes.append( self.graph[islandId] )

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
        self.inactiveIslandNodeIds
        self.inactiveIslandNodes.remove( node )
        self.inactiveIslandNodeIds.remove( node.getNodeId() )

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
            #return self.heuristic(node, island) + self.heuristic(island, self.goal)
            return self.heuristic( node, island ) + ( self.gValue( self.goal )
                                                    - self.gValue( island ) )
        else:
            return self.heuristic( node, self.goal )

    def fValue(self, node, island=None):
        return self.gValue( node ) + 10*self.hValue( node, island )

    def _inactiveIslandNodeIds(self):
        if self.inactiveIslandNodeIds:
            return self.inactiveIslandNodeIds
        else:
            self.inactiveIslandNodeIds = map( lambda n: n.getNodeId(),
                self.inactiveIslandNodes )
            return self.inactiveIslandNodeIds

    def getChildrenIslandsAndCosts(self, node):
        if(not self.graph.has_key(node.getNodeId())):
            self.graph[node.getNodeId()] = node
        point = self.getPointFromId(node.getNodeId())

        children, edgeCosts = self.getNeighbours(point[0], point[1])
        childrenNodes, prunedEdgeCosts, dummyChildrenNodes, dummyEdgeCosts = [], [], [], []


        # XXX Need to activate the island before I check the other children. In
        # case there is a dummy edge, we want to prune the children inside the
        # activation region.
        for nodeId in self._inactiveIslandNodeIds():
            if (self.distanceBetweenNodes(node.getNodeId(), nodeId) <=
            self.islandThresh):
                childIslandNode = self.graph[nodeId]
                cost = self.heuristic(node, childIslandNode)
                dummyChildrenNodes.append(childIslandNode)
                dummyEdgeCosts.append(cost)
                # XXX This is fishy!!!
                #childIslandNode.setH1( childIslandNode.getH1() + cost )
                self.activateIslandNode( childIslandNode )
                print(cost)

        for child, cost in zip(children, edgeCosts):
            nodeId = self.getIdFromPoint(child)
            if(self.graph.has_key(nodeId)):
                childNode = self.graph[nodeId]

            else:
                childNode = Node(nodeId)
                self.graph[nodeId] = childNode

            flag = 0
            for island in self.activeIslandNodes:
                if  ( (self.distanceBetweenNodes( nodeId,
                    island.getNodeId() ) <= self.islandThresh) and
                    node.getH1() < island.getH1() ):
                    flag = 1
                    break
            if flag:
                continue

            childrenNodes.append(childNode)
            prunedEdgeCosts.append(cost)
        #print (inactiveIslandNodeIds)


        return (childrenNodes, dummyChildrenNodes, prunedEdgeCosts, dummyEdgeCosts)

    def retrievePath( self, start , goal ):
        path = []
        currNode = goal
        while(currNode != start):
            path.append( currNode )
            currNode = currNode.getParent()
        return path[::-1]
