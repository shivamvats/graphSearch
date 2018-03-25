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
                if True: #not _dropIslandRegionNodes(child):
                    child.g1, child.h1 = (currNode.g1 + edgeCost), currNode.h1
                    child.setParent(currNode)
                    child.history = currNode.history
                    for region in self.env.islandRegions:
                        if currNode.getNodeId() == region.island.getNodeId():
                            child.history[currNode.getNodeId()] = True
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
                            self.goalNode, inflation=1), child))
        for child, edgeCost in zip(dummyChildren, dummyEdgeCosts):
            if currNode.gValue() + edgeCost < child.gValue():
                child.g1, child.h1 = currNode.g1, currNode.h1 + edgeCost
                child.setParent(currNode)
                print("Dummy child", self.env.fValue(child, self.goalNode,
                    inflation=1))
                Q.heappush(self.Open, (self.env.fValue(child,
                           self.goalNode, inflation=1), child))

    def _expand2(self, currNode, goal, Open, island=None):
        succs, costs = self.env.getChildrenAndCosts(currNode)
        for succ, cost in zip(succs, costs):
            if currNode.gValue() + cost <= succ.gValue():
                succ.g1 = currNode.g1 + cost
                succ.h1 = currNode.h1
                Q.heappush(Open, (self.env.fValue(succ, goal, inflation=1), succ))

    def _printDummySearchStats(self):
        islandIds = self.env.getIslandIds()
        print("Printing dummy planning related stats.")
        goal = self.env.graph[self.goalNode.getNodeId()]
        print("Goal: f = %f, g1 = %f, h1 = %f"%(self.env.fValue(goal, goal),
            goal.g1, goal.getH1()))
        for i, iid in enumerate(islandIds):
            node = self.env.graph[iid]
            print("Island%d : f = %f, gValue = %f, g1 = %f, h1 = %f"%(i,
                    self.env.fValue(node, self.goalNode), node.gValue(), node.g1,
                    node.getH1()))
        print("Nodes expanded: %d"%len(self.Closed.keys()))


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
        self.islandClosed = {}

        Q.heappush(self.Open, (self.env.fValue(startNode, goalNode, \
                inflation=1), startNode))
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
                def suspendNode(nodeId):
                    for region in self.env.islandRegions:
                        point = self.env.getPointFromId(nodeId)
                        neighbours, _ = self.env.getNeighbours(point[0],
                                point[1])
                        if region.interiorContains(point, neighbours):
                            if not (region.island.getNodeId() in
                                    currNode.history) and not (nodeId ==
                                    region.island.getNodeId()):
                                return True
                    return False
                if suspendNode(nodeId):
                    continue

                self._expand1(currNode)
                self.Closed[nodeId] = 1
                stateTimeStamps[nodeId] = (time.time(), currNode.getH())
                if viz.incrementalDisplay:
                    viz.markPoint(self.env.getPointFromId(currNode.getNodeId()), 100)
                    viz.displayImage(1)
            print("Dummy planning done.")

        _dummyPlanning()
        self._printDummySearchStats()
        viz.saveImage("dummy_planning.jpg")

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
            Q.heappush(queue, (self.env.fValue(entry, island), entry))

            #print(queue[0][0], self.inflation*self.env.heuristic(startNode,
                    #queue[0][1]))
            # Start A*
            node = entry
            print("Starting Refinement")
            while(queue[0][0] <= self.env.islandRegions[index].inflation*self.env.fValue(island,
                    goalNode) and node.getNodeId() != island.getNodeId()):
                _, node = Q.heappop(queue)
                if node.getNodeId() in self.islandClosed:
                    continue
                self._expand2(node, island, queue)
                self.islandClosed[node.getNodeId()] = 1
                if viz.incrementalDisplay:
                    viz.markPoint(self.env.getPointFromId(node.getNodeId()), 100)
                    viz.displayImage(1)

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
                        inflation=1), island))

                # Find another path to the goal.
                _dummyPlanning()

        self.stateTimeStamps = stateTimeStamps

        endTime = time.time()
        timeTaken = endTime - startTime
        print("Total time taken for planning is %f", timeTaken)
        #print(self.stateTimeStamps)
        print("Nodes expanded", len(self.Closed) + len(self.islandClosed))

        closedNodeIds = list(self.Closed.keys())
        points = map(self.env.getPointFromId, closedNodeIds)
        viz.markPoints(points, 90)
        viz.displayImage(1)
        #if currNode.getNodeId() == self.goalNode.getNodeId():
            #return 1
        #else:
            #return 0








