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

        def _expand2(self, currNode, goal, Open):
            succs, costs = self.env.getChildrenAndCosts(currNode)
            for succ, cost in zip(succs, costs):
                if currNode.gValue() + cost < succ.gValue():
                    succ.g1 = currNode.g1 + cost
                    succ.h1 = currNode.h1
                    Q.heappush(Open, (self.env.fValue(succ, goal,
                        inflation=self.inflation), succ))

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
        startTime = time.time()

        # Dummy Planning
        def _dummyPlanning():
            currNode = self.Open[0][1]
            while(len(self.Open) and currNode.getNodeId() !=
                    self.goalNode.getNodeId()):
                priority, currNode = Q.heappop(self.Open)
                nodeId = currNode.getNodeId()
                if nodeId in self.Closed:
                    continue
                try:
                    index = self.env.getIslandIds().index(nodeId)
                    self.env.expandedIslandRegions[index] = True
                except ValueError:
                    pass

                self._expand1(currNode)
                self.Closed[nodeId] = 1
                stateTimeStamps[nodeId] = (time.time(), currNode.getH())
                if viz.incrementalDisplay:
                    viz.markPoint(self.env.getPointFromId(currNode.getNodeId()), 0)
                    viz.displayImage(1)

        _dummyPlanning()

        # Refinement
        def _findFirstIsland(goalNode):
            currNode = self.env.graph[goalNode.getNodeId()]
            path = []
            while(currNode != startNode):
                path.append(currNode)
                currNode = currNode.getParent()
            # Reverse the list.
            path = path[::-1]
            # node is the first island on dummy path.
            island = next(node for node in path if node.h1 > 0)
            return (path, island)

        goalNode = self.env.graph[goalNode.getNodeId()]
        while goalNode.h1 > 0:
            # Find first island.
            path, island = _findFirstIsland(goalNode)
            entry = island.getParent()

            # h1 cost from entry to island
            h1Decrement = island.h1 - entry.h1
            index = self.env.getIslandIds().index(island.getNodeId())
            queue = self.IslandOpen[index]
            Q.heappush(queue, (self.env.fValue(entry,
                    island, inflation=self.inflation), entry))

            # Start A*
            node = entry
            while(queue[0][0] <= self.inflation*self.env.heuristic(startNode,
                queue[0][1]) and node.getNodeId() != island.getNodeId()):
                _, node = Q.heappop(queue)
                self._expand2(node, island, queue)

            def _updateNodesInPath(path, island, g1Increment, h1Decrement):
                index = path.index(island)
                for i in range(index, len(path)):
                    vertex = path[i]
                    vertex.g1 += g1Increment
                    vertex.h1 -= h1Decrement

            if node.getNodeId() == island.getNodeId():
                # Dummy path refined.
                g1Increment = island.g1 - entry.g1
                _updateNodesInPath(path, island, g1Increment, h1Decrement)
            else:
                # Alternate path being explored.
                # XXX What if goal is reached?
                minCost, minNode = queue[0]
                g1Increment = minNode.g1 - entry.g1
                # Update the g1 and h1 values of island based on the minimum of
                # queue.
                island.g1, island.h1 = minNode.g1, self.env.heuristic(minNode,
                        island)
                index = path.index(island)
                h1Decrement -= self.env.heuristic(minNode, island)
                _updateNodesInPath(path, island, g1Increment, h1Decrement)

                Q.heappush(self.Open, (self.env.fValue(island, goalNode,
                        inflation=self.inflation), island))

                # Find another path to the goal.
                _dummyPlanning()

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








