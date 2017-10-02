from env import GridEnvironment
from node import Node
import copy

class IslandClusterGridEnvironment(GridEnvironment):
    def __init__(self, envMap, rows, cols, islandClusters):
        super(IslandClusterGridEnvironment, self).__init__(envMap, rows, cols)
        self.addIslandClusters(islandClusters)
        self.activeClusterIx = -1

    def _createIslandDict(self):
        islandDict = {}
        for i in range(len(self.islandClusters)):
            for j in range(len(self.islandClusters[i])):
                # Store the position of island in the cluster.
                islandDict[self.islandClusters[i][j]] = j
        self.islandDict = islandDict
        print(self.islandClusters)
        print(self.islandDict)

    def addIslandClusters(self, islandClusters):
        self.islandClusters, self.islandClusterAvailable = [], []
        for cluster in islandClusters:
            nodeCluster = []
            for island in cluster:
                node = Node(self.getIdFromPoint(island))
                node.history.append(node)
                self.addNode(node)
                nodeCluster.append(node.getNodeId())
            self.islandClusters.append(nodeCluster)
            # XXX 1 for available and 0 for not.
            self.islandClusterAvailable.append(1)
        # Create a dict of islands with island node id as key and (cluster,
        # position) as value.
        self._createIslandDict()

        # Store exit states separately.
        # Story index of cluster to which it belongs.
        self.exitStates = {}
        for i in range(len(self.islandClusters)):
            self.exitStates[self.islandClusters[i][-1]] = i

    def deleteIslandCluster(self, index):
        del self.islandClusters[index]

    def _getNextIsland(self, cluster, node):
        #print(self.islandDict)
        if not node.history:
            # Find nearest island in the cluster.
            minDist = float("inf")
            minIx = -1
            for i in range(len(cluster)):
                dist = self.distanceBetweenNodes(node.getNodeId(),
                        cluster[i])
                if dist < minDist:
                    minDist = dist
                    minIx = i
            return minIx
        elif node.history[-1] == cluster[-1]:
            # Reached the exit state.
            return -1
        else:
            # Return the next island
            return self.islandDict[node.history[-1]] + 1

    def islandClusterHeuristic(self, index, currNode, goalNode):
        heuristicCost = 0
        cluster = self.islandClusters[index]
        islandIx = self._getNextIsland(cluster, currNode)
        print(cluster)
        print(islandIx)

        if islandIx >= 0:
            heuristicCost = (self.heuristic(currNode,
                self.graph[cluster[islandIx]]) +
                    self.heuristic(self.graph[cluster[islandIx]], goalNode))
        else:
            heuristicCost = self.heuristic(currNode, goalNode)

        #for i in range(len(cluster) - 1):
        #    start, goal = cluster[i], cluster[i+1]
        #    #print(self.heuristic(start, goal))
        #    heuristicCost += self.heuristic(start, goal)
        return heuristicCost

    def islandHeuristic(self, currNode, island, goalNode):
        if len(currNode.history) == 0:
            return self.heuristic(currNode, island) + self.heuristic(island,
                    goalNode)
        else:
            return self.heuristic(currNode, goalNode)










