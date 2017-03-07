from astar import Astar
from env import GridEnvironment, IslandGridEnvironment
from node import Node
from occupancyGrid import OccupancyGrid
from visualizer import ImageVisualizer


def main():
    """Numpy array is accessed as (r, c) while a point is (x, y). The code
    follows (r, c) convention everywhere. Hence, be careful whenever using a
    point with opencv."""
    occGrid = OccupancyGrid()
    occMap = occGrid.getMapFromImage("../data/testMapSmall.png")
    print(occMap.shape)
    #gridEnv = GridEnvironment(occMap, occMap.shape[0], occMap.shape[1])
    #gridEnv.setHeuristic(1)

    viz = ImageVisualizer(occMap)


    ###
    # Island related stuff
    ###
    islands = [(140, 100),]
    gridEnv = IslandGridEnvironment(occMap, occMap.shape[0], occMap.shape[1],
            islands)
    gridEnv.setHeuristic(1)
    gridEnv.setIslandThresh(50)
    for island in gridEnv.getIslandNodes():
        viz.drawCircle(gridEnv.getPointFromId(island.getNodeId()), 50)
        viz.displayImage(1)
        

    ###

    startPoint = (100, 20)
    goalPoint = (201, 200)
    assert(gridEnv.isValidPoint(startPoint))
    assert(gridEnv.isValidPoint(goalPoint))
    #goalPoint = (500, 50)

    #TEST
    goalId = gridEnv.getIdFromPoint(goalPoint)
    print(goalId)
    goal = gridEnv.getPointFromId(goalId)
    print(goal, goalPoint)

    startNode = Node(gridEnv.getIdFromPoint(startPoint))
    startNode.setParent(None)
    goalNode = Node(gridEnv.getIdFromPoint(goalPoint))

    gridEnv.addNode(goalNode)

    planner = Astar(gridEnv)
    planFound = planner.plan(startNode, goalNode, viz=viz)

    path = []
    if planFound:
        print("Planning successful")
        currNode = goalNode
        while(currNode != startNode):
            path.append(currNode)
            currNode = currNode.getParent()

    pathPoints = []
    for node in path:
        #print(node.getNodeId())
        pathPoints.append(gridEnv.getPointFromId(node.getNodeId()))

    viz.displayImage()
    viz.joinPointsInOrder(pathPoints, thickness=5)
    #viz.displayImage()






main()

