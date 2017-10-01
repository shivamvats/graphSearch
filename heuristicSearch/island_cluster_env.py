from env import GridEnvironment
from node import Node
import copy

class IslandClusterGridEnvironment(GridEnvironment):
    def __init__(self, envMap, rows, cols, islandClusters):
        super(IslandClusterGridEnvironment, self).__init__(envMap, rows, cols)
        self.addIslandClusters(islandClusters)
        self.activeClusterIx = -1

    def addIslandClusters(self, islandClusters):
        self.islandClusters = []
        for cluster in islandClusters:
            nodeCluster = []
            for island in cluster:
                node = Node(self.getIdFromPoint(island))
                node.history.append(node)
                self.addNode(node)
                nodeCluster.append(node)
            self.islandClusters.append(nodeCluster)
        self.availableClusters = copy.copy(self.islandClusters)

    def deleteIslandCluster(self, index):
        del self.islandClusters[index]

    def _getNextIsland(self, cluster, node):
        if not node.history:
            # Find nearest island in the cluster.
            minDist = float("inf")
            minIx = -1
            for i in range(len(cluster)):
                dist = self.distanceBetweenNodes(node.getNodeId(),
                        cluster[i].getNodeId())
                if dist < minDist:
                    minDist = dist
                    minIx = i
            return minIx
        elif len(node.history) == len(cluster):
            return -1
        else:
            # Return the next island
            return len(node.history)

    def islandClusterHeuristic(self, index, currNode, goalNode):
        print(index)
        heuristicCost = 0
        cluster = self.islandClusters[index]
        islandIx = self._getNextIsland(cluster, currNode)

        if islandIx >= 0:
            heuristicCost = (self.heuristic(currNode, cluster[islandIx]) +
                    self.heuristic(cluster[islandIx], goalNode))
        else:
            heuristicCost = self.heuristic(currNode, goalNode)

        #for i in range(len(cluster) - 1):
        #    start, goal = cluster[i], cluster[i+1]
        #    #print(self.heuristic(start, goal))
        #    heuristicCost += self.heuristic(start, goal)
        return heuristicCost

    def islandHeuristic(self, currNode, island, goalNode):
        return self.heuristic(currNode, island) + self.heuristic(island,
                goalNode)








