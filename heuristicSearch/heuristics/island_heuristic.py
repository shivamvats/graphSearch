
class IslandHeuristic():
    def __init__(self):
        pass

    def heuristic(self, currNode, goalNode, env, metric):
        def getShortestCostViaAnIsland():
            islands = env.getIslandNodes()
            costViaIsland = min (
                                map( lambda island: metric(currNode, island) +
                                    metric(island, goalNode), islands )
                                )
            return costViaIsland
        if not currNode.viaIsland:
            cost = getShortestCostViaAnIsland()
            return cost
        else:
            cost = metric( currNode, goalNode )
            return cost

