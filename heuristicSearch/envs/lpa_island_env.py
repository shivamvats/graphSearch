from heuristicSearch.graph.node import Node
from island_env import IslandGridEnvironment


class LPAIslandEnvironment(IslandGridEnvironment):

    def __init__(self, envMap, rows, cols, islandPoints):
        super(LPAIslandEnvironment, self).__init__(envMap, rows, cols,
                    islandPoints)
        # Key is endId
        self.updatedEdgeCosts = {}

    def deleteAllDummyEdges(self):
        for island in self.islandNodeIds:
            # Essentially delete this edge.
            self.updatedEdgeCosts[island] = float('inf')
        self.islandNodeIds = []
