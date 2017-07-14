import Queue as Q
import time
import timeit
import collections

class Astar(object):
    def __init__(self, env):#, fileName):
        self.env = env
        #self.fileName = fileName

    def updateG(self, node, newG):
        if(node.getG() > newG):
            node.setG(newG)
            return 1
        else:
            return 0

    def getPlanStats(self):
        return self.stateTimeStamps

    #def saveStatsToFile(self):
    #    with open(self.fileName, 'w') as f:


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
        startTime = timeit.default_timer()
        while(not openQ.empty() and currNode != self.goalNode):
            priority, currNode = openQ.get()
            nodeId = currNode.getNodeId()
            if nodeId in closed:
                continue
            stateTimeStamps[nodeId] = (timeit.default_timer(), currNode.getH())
            closed[nodeId] = 1

            #    viz.markPoint(self.env.getPointFromId(currNode.getNodeId()), 0)
            #    viz.displayImage(1)

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
                    openQ.put((child.getG() + 5*child.getH(), child))

        self.stateTimeStamps = stateTimeStamps

        endTime = timeit.default_timer()
        timeTaken = endTime - startTime
        print("Total time taken for planning is %f", timeTaken)
        #print(self.stateTimeStamps)
        print("Nodes expaneded", len(closed))

        closedNodeIds = list(closed.keys())
        points = map(self.env.getPointFromId, closedNodeIds)
        viz.markPoints(points, 90)
        viz.displayImage(1)
        if currNode.getNodeId() == self.goalNode.getNodeId():
            return 1
        else:
            return 0

class SkipEdgesAstar(Astar):
    def __init__(self, env):
        super(IslandAstar, self).__init__(env)

    def updateDummyG(self, node, newG):
        node.setHasDummyG(False)
        if(node.getG() > newG or node.checkDummyG()):
            node.setG(newG)
            return 1
        else:
            return 0

    def plan(self, startNode, goalNode, viz=None):
        self.startNode = startNode
        self.goalNode = goalNode

        self.startNode.setG(0)
        heuristicCost = self.env.heuristic(startNode, goalNode)
        startNode.setH(heuristicCost)

        openQ = Q.PriorityQueue()
        closed = []
        closed2 = []

        openQ.put((startNode.getH() + startNode.getG(), startNode))

        currNode = startNode
        while(not openQ.empty() and currNode != self.goalNode):
            priority, currNode = openQ.get()
            if currNode in closed or currNode in closed2:
                continue
            if currNode.checkDummyG():
                closed2.append(currNode)
            else:
                closed.append(currNode)
            #print(currNode.getG()+ currNode.getH())
            if currNode in self.env.activatedIslandNodes:
                print("Island node expanded", "g = ", currNode.getG())

            #if viz is not None:
            viz.markPoint(self.env.getPointFromId(currNode.getNodeId()), 0)
            viz.displayImage(1)

            children, edgeCosts, islandFound = \
            self.env.getChildrenWithIslandsAndCosts(currNode)
            if islandFound:
                # XXX Assumes only one island child and it is the last element
                # in the list.
                islandChild = children[-1]
                #print("Island node", self.env.getPointFromId(islandChild.getNodeId()))
                #print("Cost is ", edgeCosts[-1])
                if currNode in self.env.activatedIslandNodes:
                    pass
                elif self.env.ancestoryContainsNode(currNode, islandChild,
                        self.env.getIslandThresh()):
                    #print(currNode.getNodeId(), islandChild.getNodeId())
                    pass
                elif islandChild in self.env.activatedIslandNodes:
                    continue
                self.env.activateIslandNode(islandChild)
            for child, edgeCost in zip(children, edgeCosts):
                if child in closed:
                    continue

                if child.getH() == float("inf"):
                    child.setH(self.env.heuristic(child, goalNode))

                #print(currNode.getH())
                updated = self.updateG(child, currNode.getG() + edgeCost)
                if updated:
                    child.setParent(currNode)
                    #XXX What if this node is already in the open list?
                    openQ.put((child.getG() + child.getH(), child))

        print("Nodes expaneded", len(closed))

        if currNode.getNodeId() != self.goalNode.getNodeId():
            # No plan found.
            return 0

        else:
            # 1. Check if any island is in the discovered path.
            # 2. Complete the edge to the island.
            # Currently assumes only one island edge.

            currNode = goalNode
            while(currNode != startNode):
                if(currNode in self.env.activatedIslandNodes):
                    break
                currNode = currNode.getParent()
            if currNode == startNode:
                # No island node in path.
                return 1

            # currNode is an island
            pseudoGoalNode = currNode
            pseudostartNode = currNode.getParent()

            ### LOG
            print("Plan found. Beginning correction of the plan.")
            print(currNode.getNodeId())

            openQ.put((pseudostartNode.getG() + pseudostartNode.getH(),
                pseudostartNode))

            while(not openQ.empty() and currNode != self.goalNode):
                priority, currNode = openQ.get()
                print(currNode.getNodeId())
                # Previously closed nodes might need to be re-expanded.
                if currNode in closed:
                    continue
                closed.append(currNode)

                #if viz is not None:
                viz.markPoint(self.env.getPointFromId(currNode.getNodeId()), 0)
                viz.displayImage(1)

                children, edgeCosts = self.env.getChildrenAndCosts(currNode)
                for child, edgeCost in zip(children, edgeCosts):
                    if child in closed2:
                        continue

                    if child.getH() == float("inf"):
                        child.setH(self.env.heuristic(child, goalNode))

                    #print(currNode.getH())
                    # Check for dummy value is inside update function.
                    updated = self.updateDummyG(child, currNode.getG() + edgeCost)
                    if updated:
                        child.setParent(currNode)
                        #XXX What if this node is already in the open list?
                        openQ.put((child.getG() + child.getH(), child))

        if currNode.getNodeId() == self.goalNode.getNodeId():
            print("Plan found")
            return 1
        else:
            print("Planning failed")
            return 0


class IslandAstar(Astar):
    def __init__(self, env):
        super(IslandAstar, self).__init__(env)

    def plan(self, startNode, goalNode, viz=None):
        self.startNode = startNode
        self.goalNode = goalNode

        self.startNode.setG(0)
        heuristicCost = self.env.heuristic(startNode, goalNode)
        startNode.setH(heuristicCost)

        openQ = Q.PriorityQueue()
        closed = []

        openQ.put((startNode.getH() + startNode.getG(), startNode))

        currNode = startNode
        while(not openQ.empty() and currNode != self.goalNode):
            priority, currNode = openQ.get()
            if currNode in closed:
                continue
            closed.append(currNode)

            #if viz is not None:
            viz.markPoint(self.env.getPointFromId(currNode.getNodeId()), 0)
            viz.displayImage(1)

            children, edgeCosts = \
            self.env.getChildrenWithIslandsAndCosts(currNode)
            for child, edgeCost in zip(children, edgeCosts):
                if child in closed:
                    continue

                if child.getH() == float("inf"):
                    child.setH(self.env.heuristic(child, goalNode))

                #print(currNode.getH())
                updated = self.updateG(child, currNode.getG() + edgeCost)
                if updated:
                    child.setParent(currNode)
                    #XXX What if this node is already in the open list?
                    openQ.put((child.getG() + 2*child.getH(), child))

        print("Nodes expaneded", len(closed))
        if currNode.getNodeId() == self.goalNode.getNodeId():
            return 1
        else:
            return 0





