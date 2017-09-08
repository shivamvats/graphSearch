from island_search import Astar, IslandAstar, DummyEdgeAstar
from nisland_env import NIslandGridEnvironment
from node import Node
from occupancyGrid import OccupancyGrid
from visualizer import ImageVisualizer
import matplotlib.pyplot as plt
import cv2 as cv
import pickle

from utils import *

def main():
    """Numpy array is accessed as (r, c) while a point is (x, y). The code
    follows (r, c) convention everywhere. Hence, be careful whenever using a
    point with opencv."""
    occGrid = OccupancyGrid()
    occMap = occGrid.getMapFromImage("../data/complex_maze.png")
    print(occMap.shape)

    viz = ImageVisualizer(occMap)

    #islands = [(140, 100),]
    #islands = []
    #numIslands = 4
    #for i in range(numIslands):
    #    print("Click on an island")
    #    island = inputClickedPoint(occMap)
    #    islands.append(island)
    #islands = pickle.load( open( "islands_4.pkl", "rb" ) )
    islands = pickle.load( open( "islands_6.pkl", "rb" ) )
    startPoint, goalPoint = pickle.load( open( "start_goal.pkl", "rb") )

    #startPoint = (100, 20)
    #goalPoint = (201, 200)
    #print("Click start point")
    #startPoint = inputClickedPoint(occMap)
    #print("Click end point")
    #goalPoint = inputClickedPoint(occMap)
    print(startPoint, goalPoint)

    # Environment initialization
    #gridEnv = IslandGridEnvironment(occMap, occMap.shape[0], occMap.shape[1],
    #        islands)
    gridEnv = NIslandGridEnvironment(occMap, occMap.shape[0], occMap.shape[1],
            islands)

    startNode = Node(gridEnv.getIdFromPoint(startPoint))
    startNode.setParent(None)
    goalNode = Node(gridEnv.getIdFromPoint(goalPoint))
    gridEnv.addNode(goalNode)
    gridEnv.goal(goalNode)
    gridEnv.setHeuristic(0)
    gridEnv.setIslandThresh(70)

    assert(gridEnv.isValidPoint(startPoint))
    assert(gridEnv.isValidPoint(goalPoint))

    # Island visualization.
    for island in gridEnv.getIslandNodes():
        viz.drawCircle(gridEnv.getPointFromId(island.getNodeId()),
                gridEnv.islandThresh)
        viz.displayImage(1)
    cv.destroyAllWindows()

    # Planner
    #planner = IslandAstar(gridEnv)
    planner = DummyEdgeAstar( gridEnv )
    planFound = planner.plan(startNode, goalNode, viz=viz)
    #planFound = planner.plan(startNode, goalNode)

    path = []
    if planFound:
        print("Planning successful")
        currNode = goalNode
        while(currNode != startNode):
            path.append(currNode)
            currNode = currNode.getParent()
        # Reverse the list.
        path = path[::-1]

        planStateIds = map(lambda node : node.getNodeId(), path)

        pathPoints = []
        for node in path:
            #print(node.getNodeId())
            pathPoints.append(gridEnv.getPointFromId(node.getNodeId()))

        viz.displayImage()
        viz.joinPointsInOrder(pathPoints, thickness=5)
        viz.displayImage()
        cv.waitKey(0)

    #cv.imwrite(  )

main()

