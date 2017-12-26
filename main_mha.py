from heuristicSearch.planners.mhastar import MHAstar
from heuristicSearch.envs.env import GridEnvironment
from heuristicSearch.envs.occupancy_grid import OccupancyGrid
from heuristicSearch.graph.node import Node
from heuristicSearch.utils.visualizer import ImageVisualizer
from heuristicSearch.utils.utils import *

from functools import partial
import matplotlib.pyplot as plt
import cv2 as cv
import pickle
import sys


def main():
    """Numpy array is accessed as (r, c) while a point is (x, y). The code
    follows (r, c) convention everywhere. Hence, be careful whenever using a
    point with opencv."""

    folder = sys.argv[1]
    image = folder + "/image.png"
    start_goal = folder + "/start_goal.pkl"
    startPoint, goalPoint = pickle.load( open(start_goal, "rb") )

    occGrid = OccupancyGrid()
    occMap = occGrid.getMapFromImage(image)
    viz = ImageVisualizer(occMap)
    viz.incrementalDisplay = True
    print(occMap.shape)
    print(startPoint, goalPoint)

    gridEnv = GridEnvironment(occMap, occMap.shape[0], occMap.shape[1])
    # List of two heuristics.
    heuristics = (gridEnv.euclideanHeuristic, gridEnv.diagonalHeuristic)
    gridEnv.setHeuristic(heuristics)

    startNode = Node(gridEnv.getIdFromPoint(startPoint))
    startNode.setParent(None)
    goalNode = Node(gridEnv.getIdFromPoint(goalPoint))
    gridEnv.addNode(goalNode)
    gridEnv.goal(goalNode)
    assert(gridEnv.isValidPoint(startPoint))
    assert(gridEnv.isValidPoint(goalPoint))

    # Planner
    planner = MHAstar( gridEnv, w1=5, w2=5 )
    planFound = planner.plan(startNode, goalNode, viz=viz)

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
            pathPoints.append(gridEnv.getPointFromId(node.getNodeId()))

        viz.displayImage()
        #viz.joinPointsInOrder(pathPoints, thickness=5)
        viz.markPoints( pathPoints, color=100 )
        viz.displayImage()
        cv.waitKey(0)

    #cv.imwrite(  )

main()

