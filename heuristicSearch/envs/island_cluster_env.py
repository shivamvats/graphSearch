from env import GridEnvironment
from ..graph.node import Node
import copy

class IslandClusterGridEnvironment(GridEnvironment):
    def __init__(self, envMap, rows, cols, activationCenters, exitStates):
        super(IslandClusterGridEnvironment, self).__init__(envMap, rows, cols)
        # Should be a dict for fast check.
        self.activationCenters = self._addIslands(activationCenters)
        self.exitStates = self._addIslands(exitStates)
        # Add an exit state as an island when near an activation center.
        self.islands = []

    def _addIslands(self, islands):
        nodeIds = []
        for island in islands:
            node = Node(self.getIdFromPoint(island))
            self.addNode(node)
            nodeIds.append(node.getNodeId())
        return nodeIds

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

    def islandHeuristic(self, currNode, goalNode):
        if self.islands:
            island = self.islands[-1]
            if self.islands[-1] not in currNode.history:
                return ( self.heuristic(currNode, self.graph[island]) +
                        self.heuristic(self.graph[island],
                        goalNode) )
            else:
                return self.heuristic(currNode, goalNode)
        else:
            return self.heuristic(currNode, goalNode)










