import Queue as Q
import time
import timeit
import collections

class Astar(object):
    def __init__(self, env, inflation=10):
        self.env = env
        self.inflation = inflation

    def updateG(self, node, newG):
        if(node.getG() > newG):
            node.setG(newG)
            return 1
        else:
            return 0

    def getPlanStats(self):
        return self.stateTimeStamps

    #@profile
    def plan(self, startNode, goalNode, viz=None):
        self.startNode = startNode
        self.goalNode = goalNode

        # Ordered List of expanded sates and their timestamps.
        stateTimeStamps = collections.OrderedDict()

        self.startNode.setG(0)
        heuristicCost = self.env.heuristic(startNode, goalNode)
        startNode.setH(heuristicCost)

        openQ = Q.PriorityQueue()
        # Using a dictionary 'cos list has slow lookup.
        closed = {}

        openQ.put((startNode.getH() + startNode.getG(), startNode))

        currNode = startNode
        startTime = time.time()
        while(not openQ.empty() and currNode.getNodeId() !=
                self.goalNode.getNodeId()):
            priority, currNode = openQ.get()
            nodeId = currNode.getNodeId()
            if nodeId in closed:
                continue
            stateTimeStamps[nodeId] = (time.time(), currNode.getH())
            closed[nodeId] = 1

            if viz.incrementalDisplay:
                viz.markPoint(self.env.getPointFromId(currNode.getNodeId()), 100)
                viz.displayImage(1)

            children, edgeCosts = \
                    self.env.getChildrenAndCosts(currNode)
            for child, edgeCost in zip(children, edgeCosts):
                if child.getNodeId() in closed:
                    continue

                if child.getH() == float("inf"):
                    child.setH(self.env.heuristic(child, goalNode))

                updated = self.updateG(child, currNode.getG() + edgeCost)
                if updated:
                    child.setParent(currNode)
                    #XXX What if this node is already in the open list?
                    openQ.put((child.getG() + self.inflation*child.getH(), child))

        self.stateTimeStamps = stateTimeStamps

        endTime = time.time()
        timeTaken = endTime - startTime
        print("Total time taken for planning is %f", timeTaken)
        #print(self.stateTimeStamps)
        print("Nodes expanded", len(closed))

        closedNodeIds = list(closed.keys())
        points = map(self.env.getPointFromId, closedNodeIds)
        viz.markPoints(points, 90)
        viz.displayImage(1)
        if currNode.getNodeId() == self.goalNode.getNodeId():
            return 1
        else:
            return 0








