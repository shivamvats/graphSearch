#import Queue as Q
import heapq as Q
import time
import timeit
import collections

class DynamicMHAstar():
    # XXX Assumes there is only one inadmissible heuristic at any given time
    # and that is the dynamically generated heuristic.
    def __init__(self, env, w1, w2):
        self.env = env
        self.anchorQ = []
        # Using a dictionary 'cos list has slow lookup.
        self.closed = {}

        self.w1 = w1
        self.w2 = w2
        self.inadQs = []
        self.inadHeurs = []
        self.numInadHeurs = 0

    def updateG(self, node, newG):
        if(node.getG() > newG):
            node.setG(newG)
            return 1
        else:
            return 0

    def addHeuristic(self, heuristic):
        """Add a new heuristic and a separate Open queue for it."""
        self.inadQs.append( [] )
        self.inadHeurs.append( heuristic )
        self.numInadHeurs += 1

    def addDynamicHeuristic(self, heuristic):
        """Dynamic MHA* specific function.
        i is the index of the heuristic.
        heuristic is a function that takes in start and goal nodes and returns
        the heuristic cost."""
        # XXX I am copying the open list of anchor. Not sure if this is the
        # right thing to do in MHA*. In my case, I am not using any
        # inadmissible heuristic, so it is fine.
        self.addHeuristic(heuristic)
        anchorOpenNodes = [ y for x, y in self.anchorQ ]
        self.inadQs[-1] = [ (node.getG() + heuristic(node, self.goalNode), node) for node in
                anchorOpenNodes ]
        Q.heapify( self.inadQs[-1] )

    def deleteHeuristic(self, index):
        del self.inadQs[index]
        del self.inadHeurs[index]
        self.numInadHeurs -= 1

    def deleteDynamicHeuristic(self):
        # Assumes no other inadmissible heuristic is being used.
        self.deleteHeuristic(0)

    def getPlanStats(self):
        return self.stateTimeStamps

    def expand(self, queue, currNode):
        children, edgeCosts = \
        self.env.getChildrenAndCosts(currNode)
        for child, edgeCost in zip(children, edgeCosts):
            if child.getH() == float("inf"):
                child.setH(self.env.heuristic(child, goalNode))
            updated = self.updateG(child, currNode.getG() + edgeCost)
            if updated:
                child.setParent(currNode)
                #XXX What if this node is already in the open list?
                Q.heappush( queue, (child.getG() + self.w1 * child.getH(), child) )
        #viz.markPoint(self.env.getPointFromId(currNode.getNodeId()), 0)
        #viz.displayImage(1)

    def checkIfNearMinimum(currNode):
        """Only checks the distance from the first island in the cluster."""
        for i, cluster in enumerate(self.islandClusters):
            if self.distanceBetweenNodes(currNode, cluster[0]) <=
                    self.ISLANDTHRESH:
                return i
        return -1

    #@profile
    def plan(self, startNode, goalNode, viz=None):
        self.startNode = startNode
        self.goalNode = goalNode

        # Ordered List of expanded sates and their timestamps.
        stateTimeStamps = collections.OrderedDict()

        self.startNode.setG(0)
        self.goalNode.setG( float("inf") )
        heuristicCost = self.env.heuristic(startNode, goalNode)
        startNode.setH(heuristicCost)

        Q.heappush( anchorQ, ( startNode.getG() + self.w1 * startNode.getH(), startNode) )

        currNode = startNode
        startTime = time.time()
        while( len(anchorQ) ):
            ANCHORSEARCH = 0
            for i in range(self.numInadHeurs):
                def _dynamicExpansion():
                    minKey = self.inadQs[i][0][0]
                    if minKey <= self.w2 * self.anchorQ[0][0]:
                        if self.goalNode.getG() <= minKey:
                            return 1
                        priority, currNode = Q.heappop( self.inadQs[i] )
                        nodeId = currNode.getNodeId()
                        stateTimeStamps[nodeId] = (time.time(), currNode.getH())
                        self.closed[nodeId] = 1
                        self.expand(self.inadQs[i], currNode)
                        # Check if the currNode is an exit state of the
                        # cluster.
                        if currNode.getNodeId() == self.islandClusters[
                                self.activeClusterIx][-1].getNodeId():
                            self.deleteDynamicHeuristic()
                            del self.islandClusters[self.activeClusterIx]
                            self.activeClusterIx = -1
                    else:
                        ANCHORSEARCH = 1
                        return 0

                if( _inadExpansion() )
                    break

            if not(self.numInadHeurs) or ANCHORSEARCH:
                def _anchorExpansion():
                    if self.goalNode.getG() <= self.anchorQ[0][0]:
                        return 1
                    priority, currNode = Q.heappop( self.anchorQ )
                    index = checkIfNearMinimum()
                    if (index >= 0):
                        # Search inside an activation region.
                        self.addDynamicHeuristic(
                                partial(self.env.islandClusterHeuristic, index) )
                        self.env.activeClusterIx = index

                    nodeId = currNode.getNodeId()
                    stateTimeStamps[nodeId] = (time.time(), currNode.getH())
                    self.closed[nodeId] = 1
                    self.expand(self.anchorQ, currNode)
                    return 0

                if( _anchorExpansion() )
                    break

        self.stateTimeStamps = stateTimeStamps

        endTime = time.time()
        timeTaken = endTime - startTime
        print("Total time taken for planning is %f", timeTaken)
        #print(self.stateTimeStamps)
        print("Nodes expaneded", len(self.closed))

        closedNodeIds = list(self.closed.keys())
        points = map(self.env.getPointFromId, closedNodeIds)
        viz.markPoints(points, 90)
        viz.displayImage(1)
        return 0

