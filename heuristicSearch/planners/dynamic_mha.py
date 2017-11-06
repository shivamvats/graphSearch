#import Queue as Q
import heapq as Q
import time
import timeit
import collections
from functools import partial

class DynamicMHAstar():
    # XXX Assumes there is only one inadmissible heuristic at any given time
    # and that is the dynamically generated heuristic.
    def __init__(self, env, w1, w2):
        self.env = env

        self.w1 = w1
        self.w2 = w2
        # The convention is that the first element in these lists correspond to
        # the anchor heuristic.
        self.queues = {}
        self.heurs = {}
        self.numHeurs = 0

        # Using a dictionary 'cos list has slow lookup.
        self.anchorClosed = {}

    def updateG(self, node, newG):
        if(node.getG() > newG):
            node.setG(newG)
            return 1
        else:
            return 0

    def addHeuristic(self, heuristic, index):
        """Add a new heuristic and a separate Open queue for it."""
        self.queues[index] = []
        self.heurs[index] = heuristic
        self.numHeurs += 1

    def addDynamicHeuristic(self, heuristic, index):
        """Dynamic MHA* specific function.
        'index' is the index of the heuristic.
        'heuristic' is a function that takes in start and goal nodes and returns
        the heuristic cost."""
        self.addHeuristic(heuristic, index)
        anchorOpenNodes = [ y for x, y in self.queues[0] ]
        self.queues[index] = [ (node.getG() + self.w1 * heuristic(node, self.goalNode), node) for node in
                anchorOpenNodes ]
        Q.heapify( self.queues[index] )
        # If using closed, then we will need a list of closed queues.
        #self.dynamicClosed = {}

    def deleteHeuristic(self, index):
        self.queues[index] = None
        self.heurs[index] = None
        self.numInadHeurs -= 1

    def getPlanStats(self):
        return self.stateTimeStamps

    def expand(self, queue, currNode):
        children, edgeCosts = \
                self.env.getChildrenAndCosts(currNode)
        for child, edgeCost in zip(children, edgeCosts):
            updated = self.updateG(child, currNode.getG() + edgeCost)
            if updated:
                child.setParent(currNode)
                if currNode.getNodeId() in self.env.exitStates:
                    child.history = currNode.history
                    child.history[currNode.getNodeId()] = 1
                else:
                    child.history = currNode.history
                #XXX What if this node is already in the open list?
                if not child in self.anchorClosed:
                    Q.heappush( self.anchorQ, (child.getG() + self.w1 *
                        self.env.islandHeuristic(child, self.goalNode), child) )
                    #if self.numInadHeurs > 0 and not child in self.dynamicClosed:
                    #    dynamicKey = child.getG() + ( self.w1 *
                    #            self.inadHeurs[-1](child,
                    #                self.goalNode) )
                    #    anchorKey = child.getG() + ( self.w1 *
                    #            self.env.heuristic(child, self.goalNode) )
                    #    if dynamicKey <= self.w2 * anchorKey:
                    #        Q.heappush( self.inadQs[0], (dynamicKey, child) )
                # Update the other queue
        #self.viz.markPoint(self.env.getPointFromId(currNode.getNodeId()), 0)
        #self.viz.displayImage(1)

    def checkIfNearMinimum(self, currNode):
        """Only checks the distance from the first island in the cluster."""
        for i, center in enumerate(self.env.activationCenters):
            if self.env.exitStates[i] not in self.env.islands:
                if (self.env.distanceBetweenNodes(currNode.getNodeId(),
                    center) <=
                        self.env.ISLANDTHRESH):
                    return i
        return -1

    def createNewQueue(self, currNode):
        index = self.checkIfNearMinimum(currNode)
        if (index >= 0):
            # Search inside an activation region.
            print(index)
            self.addDynamicHeuristic(
                    partial(self.env.islandClusterHeuristic, index),
                    self.numInadHeurs )
            for island in self.env.islandClusters[index]:
                    self.viz.drawCircle(self.env.getPointFromId(island),
                            3, (100, 100, 100), -1)
            self.env.islandClusterAvailable[index] = 0
            #self.env.availableClusters.remove(self.env.availableClusters[index])

    #@profile
    def plan(self, startNode, goalNode, viz=None):
        self.startNode = startNode
        self.goalNode = goalNode
        self.viz = viz

        # Ordered List of expanded sates and their timestamps.
        stateTimeStamps = collections.OrderedDict()

        self.startNode.setG(0)
        self.goalNode.setG( float("inf") )
        # Add the anchor queue and heuristic.
        self.queues[0] = []
        self.heurs[0] = self.env.heuristic
        anchorQ = self.queue[0]

        heuristicCost = self.env.heuristic(startNode, goalNode)
        startNode.setH(heuristicCost)
        Q.heappush( anchorQ, ( startNode.getG() + self.w1 * startNode.getH(), startNode) )

        currNode = startNode
        startTime = time.time()
        while( len(self.anchorQ) ):
            ANCHORSEARCH = 1
            #for i in range(self.numInadHeurs):
            #    def _dynamicExpansion():
            #        minKey = self.inadQs[i][0][0]
            #        if minKey <= self.w2 * self.anchorQ[0][0]:
            #            if self.goalNode.getG() <= minKey:
            #                return 1
            #            #print(minKey, self.anchorQ[0][0])
            #            priority, currNode = Q.heappop( self.inadQs[i] )
            #            #print(priority - currNode.getG(),
            #            #        self.w1*self.env.heuristic(currNode, self.goalNode))
            #            nodeId = currNode.getNodeId()
            #            stateTimeStamps[nodeId] = (time.time(), currNode.getH())
            #            self.dynamicClosed[nodeId] = 1
            #            self.expand(self.inadQs[i], currNode)
            #            self.createNewQueue(currNode)
            #            #Check if the currNode is an exit state of the
            #            #cluster.
            #            #print("distance", self.env.distanceBetweenNodes(currNode.getNodeId(), self.env.islandClusters[
            #            #        self.env.activeClusterIx][-1].getNodeId()))
            #            #if self.env.distanceBetweenNodes(currNode.getNodeId(), self.env.islandClusters[
            #            #        self.env.activeClusterIx][-1].getNodeId()) < 2:
            #            #if currNode.getNodeId() in self.env.exitStates:
            #            #    self.deleteDynamicHeuristic()
            #            #    #del self.env.islandClusters[self.env.activeClusterIx]
            #            #    self.env.islandClusterAvailable[self.env.exitStates[currNode.getNodeId()]] = 0
            #            #    self.env.activeClusterIx = -1
            #        else:
            #            ANCHORSEARCH = 1
            #            return 0

            #    if( _dynamicExpansion() ):
            #        break

            if not(self.numInadHeurs) or ANCHORSEARCH:
                def _anchorExpansion():
                    if self.goalNode.getG() <= self.anchorQ[0][0]:
                        return 1
                    priority, currNode = Q.heappop( self.anchorQ )

                    nodeId = currNode.getNodeId()
                    stateTimeStamps[nodeId] = (time.time(), currNode.getH())
                    self.anchorClosed[nodeId] = 1
                    self.expand(self.anchorQ, currNode)
                    index = self.checkIfNearMinimum(currNode)
                    if index >= 0:
                        self.env.islands += [self.env.exitStates[index]]
                    #self.createNewQueue(currNode)
                    return 0

                if( _anchorExpansion() ):
                    break

        self.stateTimeStamps = stateTimeStamps

        for island in self.env.islands:
            self.viz.drawCircle(self.env.getPointFromId(island),
                    3, (100, 100, 100), -1)

        endTime = time.time()
        timeTaken = endTime - startTime
        print("Total time taken for planning is %f", timeTaken)
        #print(self.stateTimeStamps)
        print("Nodes expanded", len(self.anchorClosed)) #+
                #len(self.dynamicClosed))

        closedNodeIds = ( list(self.anchorClosed.keys()))# +
                #list(self.dynamicClosed.keys()) )
        points = map(self.env.getPointFromId, closedNodeIds)
        self.viz.markPoints(points, 90)
        self.viz.displayImage(1)
        return 0

