from heuristicSearch.graph.node import Node
from heuristicSearch.graph.island_region import IslandRegion
from env import GridEnvironment


class MultiIslandGridEnvironment(GridEnvironment):
    """Implements a planning environment for a multi-island A* search."""

    def __init__(self, envMap, rows, cols):
        super(MultiIslandGridEnvironment, self).__init__(envMap, rows, cols)

    @property
    def islandRegions(self):
        return self._islandRegions

    @islandRegions.setter
    def islandRegions(self, regions):
        self._islandRegions = regions
        for islandRegion in self._islandRegions:
            islandId = islandRegion.island.getNodeId()
            self.addNode(islandId)
        #self.islandNodeIds = []
        #self.activatedIslandNodes = []

        #for point in islandPoints:
        #    islandId = self.getIdFromPoint(point)
        #    self.islandNodeIds.append(islandId)
        #    if not self.graph.has_key(islandId):
        #        self.graph[islandId] = Node(islandId)

    def getIslandNodeIds(self):
        nodeIds = map(lambda x: x.island.getNodeId(),
                self._islandRegions)
        return nodeIds

    def activateIslandNode(self, node):
        if node not in self.activatedIslandNodes:
            self.activatedIslandNodes.append(node)

    def getChildrenWithIslandsAndCosts(self, node):
        self.addNode(node.getNodeId())
        point = self.getPointFromId(node.getNodeId())

        children_, edgeCosts_ = self.getNeighbours(point[0], point[1])
        children, edgeCosts = [], []
        for child, cost in zip(children_, edgeCosts_):
            if not any(region.contains(child) for region in
                    self.islandRegions):
                children.append(child), edgeCosts.append(cost)
        childrenNodes = []
        for i, child in enumerate(children):
            nodeId = self.getIdFromPoint(child)
            self.addNode(nodeId)
            childNode = self.graph[nodeId]
            childrenNodes.append(childNode)

        dummyChildrenNodes, dummyEdgeCosts = [], []
        flag = 0
        # TODO Do not return already used (inactive) islands.
        for region in self._islandRegions:
            if region.contains(self.getPointFromId(node.getNodeId())):
                nodeId = region.island.getNodeId()
                childNode = self.graph[nodeId]
                cost = self.heuristic(node, childNode)
                dummyChildrenNodes.append(childNode)
                dummyEdgeCosts.append(cost)
        return ((childrenNodes, edgeCosts), (dummyChildrenNodes,
                dummyEdgeCosts))

    def hValue(self, node, goal, inflation, island=None):
        """Calculate heuristic via island if provided and cost is within
        suboptimality."""
        if island and self.heuristic(node, island) + self.heuristic(island, \
                goal) <= inflation*self.heuristic(node, goal):
            return self.heuristic(node, island) + self.heuristic(island, goal)
        else:
            return self.heuristic(node, goal)

    def fValue(self, node, goal, island=None):
        return node.gValue() + self.hValue(node, goal, island)
