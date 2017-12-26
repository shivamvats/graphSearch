#import Queue as Q
import heapq as Q
import time
import timeit
import collections
from functools import partial

class MHAstar():
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
        self.inadClosed = {}

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

    def getPlanStats(self):
        return self.stateTimeStamps

    def expand(self, currNode, queueIndex):
        # Delete currNode from all queues.
        Q.heappop( self.queues[queueIndex] )
        for i in range( self.numHeurs ):
            if i == queueIndex:
                continue
            else:
                # XXX Seems quite expensive.
                # SBPL has its special queue.
                queue = self.queues[queueIndex]
                for i, item in enumerate(queue):
                    if item[1].getNodeId() == currNode.getNodeId:
                        del queue[i]
                        Q.heapify(queue)
        children, edgeCosts = \
                self.env.getChildrenAndCosts(currNode)
        for child, edgeCost in zip(children, edgeCosts):
            updated = self.updateG(child, currNode.getG() + edgeCost)
            if updated:
                child.setParent(currNode)
                if not child in self.anchorClosed:
                    Q.heappush( self.anchorQ, (self._key(child, self.goalNode, 0), child) )
                    if not child in self.inadClosed:
                        for i in range(1, self.numHeurs):
                            if (self._key(child, self.goalNode, i) <=
                                    self.w2*self._key(child, self.goalNode, 0)):
                                Q.heappush( self.queues[i], (self._key(child,
                                        self.goalNode, i), child) )

        self.viz.markPoint(self.env.getPointFromId(currNode.getNodeId()), 0)
        self.viz.displayImage(1)

    def _key(self, node, goal, index):
        return node.getG() + self.w1*self.env.heuristic[index](node, goal)

    def _queueMinkey(self, index):
        return self.queues[index][0][0]

    def plan(self, startNode, goalNode, viz=None):
        self.startNode = startNode
        self.goalNode = goalNode
        self.viz = viz

        # Ordered List of expanded sates and their timestamps.
        stateTimeStamps = collections.OrderedDict()
        self.startNode.setG(0)
        self.goalNode.setG( float("inf") )

        self.numHeurs = len(self.env.heuristic)
        # Assume that the first heuristic is the anchor heuristic.
        # Initialize queues and insert start in all queues.
        for i in range(self.numHeurs):
            self.queues[i] = []
            Q.heappush( self.queues[i], ( self._key(startNode, goalNode, i), startNode) )
        self.anchorQ = self.queues[0]

        currNode = startNode
        startTime = time.time()
        while( len(self.anchorQ) ):
            # Round robin.
            for i in range(self.numHeurs):
                if self._queueMinkey(i) <= self.w2*self._queueMinkey(0):
                    if goalNode.getG() <= self._queueMinkey(i):
                        if goalNode.getG() < float('infinity'):
                            return 0
                    else:
                        priority, currNode = self.queues[i][0]
                        self.expand(currNode, i)
                        nodeId = currNode.getNodeId()
                        self.inadClosed[nodeId] = 1
                        stateTimeStamps[nodeId] = (time.time(), currNode.getH())
                else:
                    if self.goalNode.getG() <= self.anchorQ[0][0]:
                        if goalNode.getG() < float('infinity'):
                            return 0
                    else:
                        priority, currNode = self.queues[i][0]
                        self.expand(currNode, i)
                        nodeId = currNode.getNodeId()
                        self.anchorClosed[nodeId] = 1
                        stateTimeStamps[nodeId] = (time.time(), currNode.getH())

        self.stateTimeStamps = stateTimeStamps
        endTime = time.time()
        timeTaken = endTime - startTime
        print("Total time taken for planning is %f", timeTaken)
        #print(self.stateTimeStamps)
        #print("Nodes expanded", len(self.anchorClosed)) #+
                #len(self.dynamicClosed))
        points = map(self.env.getPointFromId, closedNodeIds)
        self.viz.markPoints(points, 90)
        self.viz.displayImage(1)
        return 0

