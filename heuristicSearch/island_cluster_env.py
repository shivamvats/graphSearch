from env import GridEnvironment
from node import Node

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
                self.addNode(node)
                nodeCluster.append(node)
            self.islandClusters.append(nodeCluster)

    def deleteIslandCluster(self, index):
        del self.islandClusters[index]

    def islandClusterHeuristic(self, index, currNode, goalNode):
        heuristicCost = 0
        cluster = [currNode] + self.islandClusters[index] + [goalNode]
        for i in range(len(cluster) - 1):
            start, goal = cluster[i], cluster[i+1]
            heuristicCost += self.heuristic(start, goal)
        return heuristicCost







