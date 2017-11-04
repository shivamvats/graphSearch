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

    def setIslandEpsilon( self, epsilon ):
        self.islandEpsilon = epsilon

    def setSearchEpsilon( self, epsilon ):
        self.searchEpsilon = epsilon

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

    def hValue1(self, node):
        """To be used in the Dummy Planning Phase"""
        # XXX Ideally, the node should remember its h-value so that it can be used
        # in the Refinement stage. Hence, this should use node.h instead of the
        # heuristic function.
        return self.heuristic( node, self.goal )
        #return node.getH()

    def hValue2(self, node, island=None):
        """Currently island may be just a single node, but this should be extended
        to handle a set of islands."""
        # XXX Ideally, the node should remember its h-value so that it can be used
        # in the Refinement stage. Hence, this should use node.h instead of the
        # heuristic function.
        if island:
            # XXX Only valid for Refinement stage as goal.gValue = inf in the
            # first phase.
            #return self.heuristic(node, island) + self.heuristic(island, self.goal)
            return self.heuristic( node, island ) + ( self.gValue( self.goal )
                                                    - self.gValue( island ) )
        else:
            # Assuming that the g costs are optimal, this difference is an
            # underestimate.
            return self.gValue( self.goal ) - self.gValue( node )
            #return node.getH()

    def hValue3(self, node):
        """Calculate the heuristic by taking a min over all heuristic costs via
        the active islands."""
        hCost = float("inf")
        for island in self.activeIslandNodes:
            cost = self.hValue2(node, island)
            if cost <= self.islandEpsilon*self.hValue1(node) and cost < hCost:
                hCost = cost
        return min( hCost, self.hValue1(node) )

    def hValue4(self, node, island):
        # XXX Ideally, should be using the updated hvalue (or gValue(goal) -
        # gValue(node)) calculated after dummy planning phase. However, I am
        # not sure if these are being correctly maintained.
        #
        #if self.hValue2(node, island) <= (self.islandEpsilon *
        #        self.hValue(node) ):
        #    return self.gValue + self.hValue2(node, island)
        viaIsland = self.heuristic(node, island) + self.heuristic(island, self.goal) 
        viaIsland *= self.searchEpsilon
        if viaIsland <= self.islandEpsilon * self.heuristic(node, self.goal):
            return viaIsland
        else:
            return self.heuristic( node, self.goal )

    def fValue1(self, node):
        return self.gValue( node ) + self.searchEpsilon*self.hValue1( node )

    def fValue2(self, node, island=None):
        """Weighted f value"""
        return self.gValue( node ) + self.searchEpsilon*self.hValue2( node, island )

    def fValue3(self, node):
        """Return the min fValue via all islands."""
        return self.gValue( node ) + self.hValue3( node)

    def fValue4(self, node, island):
        """Via the island if not worse than islandEpsilon times the direct
            route"""
        # XXX Ideally, should be using the updated hvalue (or gValue(goal) -
        # gValue(node)) calculated after dummy planning phase. However, I am
        # not sure if these are being correctly maintained.
        #
        #if self.hValue2(node, island) <= (self.islandEpsilon *
        #        self.hValue(node) ):
        #    return self.gValue + self.hValue2(node, island)
        return self.gValue(node) + self.hValue4(node, island)


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
        print(goal.getNodeId())
        print(start.getNodeId())
        while(currNode != start):
            path.append( currNode )
            currNode = currNode.getParent()
        return path[::-1]
