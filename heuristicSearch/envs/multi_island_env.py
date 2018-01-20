from heuristicSearch.graph.node import Node
from heuristicSearch.graph.island_region import IslandRegion
from env import GridEnvironment


class MultiIslandGridEnvironment(GridEnvironment):
    """Implements a planning environment for a multi-island A* search."""

    def __init__(self, envMap, rows, cols, islandRegions):
        super(MultiIslandGridEnvironment, self).__init__(envMap, rows, cols)
        self.islandRegions = islandRegions

    @property
    def islandRegions(self):
        return self._islandRegions

    @property.setter
    def islandRegions(self, regions):
        self._islandRegions = regions
        for islandRegion in self._islandRegions:
            islandId = self.getIdFromPoint(islandRegion.island)
            self.addNode(islandId)
        #self.islandNodeIds = []
        #self.activatedIslandNodes = []

        #for point in islandPoints:
        #    islandId = self.getIdFromPoint(point)
        #    self.islandNodeIds.append(islandId)
        #    if not self.graph.has_key(islandId):
        #        self.graph[islandId] = Node(islandId)

    def getIslandNodeIds(self):
        nodeIds = map(lambda x: self.getIdFromPoint(x.island),
                self._islandRegions)
        return nodeIds

    def activateIslandNode(self, node):
        if node not in self.activatedIslandNodes:
            self.activatedIslandNodes.append(node)

    def getChildrenWithIslandsAndCosts(self, node):
        self.addNode(node.getNodeId())
        point = self.getPointFromId(node.getNodeId())

        children, edgeCosts = self.getNeighbours(point[0], point[1])
        childrenNodes = []
        for child in children:
            nodeId = self.getIdFromPoint(child)
            self.addNode(nodeId)
            childNode = self.graph[nodeId]
            childrenNodes.append(childNode)

        dummyChildrenNodes, dummyEdgeCosts = [], []
        flag = 0
        # TODO Do not return already used (inactive) islands.
        for region in self._islandRegions:
            if region.contains(self.getPointFromId(node.getNodeId())):
                nodeId = self.getIdFromPoint(region.island)
                childNode = self.graph[nodeId]
                cost = self.heuristic(node, childNode)
                dummyChildrenNodes.append(childNode)
                dummyEdgeCosts.append(cost)
        #if node.checkDummyG():
        #    for childNode in childrenNodes:
        #        childNode.setHasDummyG(True)

        return ((childrenNodes, edgeCosts), (dummyChildrenNodes,
                dummyEdgeCosts))
