from astar import *
import heapq as Q

class MultiIslandAstar(Astar):
    def __init__(self, env, inflation=10):
        super(MultiIslandAstar, self).__init__(env, inflation)

    def _expand1(self, currNode):
        """Dummy Expansion"""
        [children, edgeCosts], [dummyChildren, dummyEdgeCosts] = \
                self.env.getChildrenWithIslandsAndCosts(currNode)
        # TODO Nodes falling inside an influence region should be inserted into
        # the corresponding open list.
        for child, edgeCost in zip(children, edgeCosts):
            #if child.getNodeId() in self.Closed:
                #continue
            if currNode.gValue() + edgeCost < child.gValue():
                def _dropIslandRegionNodes(child):
                    for i, region in enumerate(self.env.islandRegions):
                        if (self.env.activatedIslandRegions[i] and not self.env.expandedIslandRegions[i] and
                                region.contains(self.env.getPointFromId(child.getNodeId()))):
                            return True
                if not _dropIslandRegionNodes(child):
                    child.g1, child.h1 = (currNode.g1 + edgeCost), currNode.h1
                    child.setParent(currNode)
                # Insert child into an island Open list if it falls inside its
                # influence region.
                #def _insertIntoIslandOpen(child):
                #    inserted = False
                #    for i, region in enumerate(self.env.islandRegions):
                #        if self.env.activatedIslandRegions[i] and region.contains(self.env.getPointFromId(child.getNodeId())):
                #            Q.heappush(self.IslandOpen[i], (self.env.fValue(child,
                #                    self.goalNode, inflation=self.inflation, island=region.island), child))
                #            inserted = True
                #    return inserted

                #if not _insertIntoIslandOpen(child):
                    Q.heappush(self.Open, (self.env.fValue(child, \
                            self.goalNode, inflation=self.inflation), child))
        for child, edgeCost in zip(dummyChildren, dummyEdgeCosts):
            if currNode.gValue() + edgeCost < child.gValue():
                child.g1, child.h1 = currNode.g1, currNode.h1 + edgeCost
                child.setParent(currNode)
                print("Dummy child", self.env.fValue(child, self.goalNode,
                    inflation=self.inflation))
                Q.heappush(self.Open, (self.env.fValue(child,
                           self.goalNode, inflation=self.inflation), child))

    #@profile
    def plan(self, startNode, goalNode, viz=None):
        self.startNode = startNode
        self.goalNode = goalNode
        print(goalNode.getNodeId())

        # Ordered List of expanded sates and their timestamps.
        stateTimeStamps = collections.OrderedDict()

        self.startNode.g1 = 0
        self.IslandOpen=[ [] for region in self.env.islandRegions ]
        self.Open = []
        # Using a dictionary 'cos list has slow lookup.
        self.Closed = {}

        Q.heappush(self.Open, (self.env.fValue(startNode, goalNode, \
                inflation=self.inflation), startNode))
        currNode = startNode
        startTime = time.time()

        # Dummy Planning
        counter = 0
        while(len(self.Open) and currNode.getNodeId() !=
                self.goalNode.getNodeId()):
            priority, currNode = Q.heappop(self.Open)
            nodeId = currNode.getNodeId()
            if nodeId in self.Closed:
                continue
            try:
                index = [region.island.getNodeId() for region in
                        self.env.islandRegions].index(nodeId)
                self.env.expandedIslandRegions[index] = True
            except ValueError:
                pass

            self._expand1(currNode)
            self.Closed[nodeId] = 1
            stateTimeStamps[nodeId] = (time.time(), currNode.getH())
            if viz.incrementalDisplay:
                viz.markPoint(self.env.getPointFromId(currNode.getNodeId()), 0)
                viz.displayImage(1)
            counter += 1
            #if counter == 5:
                #return

        self.stateTimeStamps = stateTimeStamps

        endTime = time.time()
        timeTaken = endTime - startTime
        print("Total time taken for planning is %f", timeTaken)
        #print(self.stateTimeStamps)
        print("Nodes expanded", len(self.Closed))

        closedNodeIds = list(self.Closed.keys())
        points = map(self.env.getPointFromId, closedNodeIds)
        viz.markPoints(points, 90)
        viz.displayImage(1)
        if currNode.getNodeId() == self.goalNode.getNodeId():
            return 1
        else:
            return 0








