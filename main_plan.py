from heuristicSearch.planners.astar import Astar
from heuristicSearch.envs.env import GridEnvironment
from heuristicSearch.graph.node import Node
from heuristicSearch.envs.occupancy_grid import OccupancyGrid
from heuristicSearch.utils.visualizer import ImageVisualizer
from heuristicSearch.utils.utils import *

import matplotlib.pyplot as plt
import cv2 as cv
import sys
import pickle

"""
Note: Numpy array is accessed as (r, c) while a point is (x, y). The
code follows (r, c) convention everywhere. Hence, be careful whenever
using a point with opencv.
"""
def saveStats(stats):
    with open("state_time.csv", "w") as f:
        keys, vals = stats.keys(), stats.values()
        for key, val in zip(keys, vals):



def main():
    """
    The main function that sets up the environment and calls the planner.
    Each planning run has three components:

    * Environment
    * Graph
    * Planner

    The Environment class contains details about the specific planning problem,
    eg: a 2D map for 2D planning. Its primary role is to implement a
    `getChildren` method that returns the successors of each node. This helps
    in building the graph implicitly.

    Each vertex in the graph is an instance of `Node` class. It stores details
    specific to the node.

    An Astar instance runs on the graph so created.
    """

    folder = sys.argv[1]
    image = folder + "/image.png"
    start_goal = folder + "/start_goal.pkl"
    startPoint, goalPoint = pickle.load( open(start_goal, "rb") )

    # Build the planning environment.
    occGrid = OccupancyGrid()
    occMap = occGrid.getMapFromImage(image)
    print(occMap.shape)
    gridEnv = GridEnvironment(occMap, occMap.shape[0], occMap.shape[1])
    gridEnv.setHeuristicType(0)
    def abcissaHeuristic(currNode, goalNode):
        currPoint = gridEnv.getPointFromId(currNode.getNodeId())
        goalPoint = gridEnv.getPointFromId(goalNode.getNodeId())
        return abs(currPoint[0] - goalPoint[0])

    def ordinateHeuristic(currNode, goalNode):
        currPoint = gridEnv.getPointFromId(currNode.getNodeId())
        goalPoint = gridEnv.getPointFromId(goalNode.getNodeId())
        return abs(currPoint[1] - goalPoint[1])
    #gridEnv.setHeuristic(ordinateHeuristic)

    # For visualization.
    viz = ImageVisualizer(occMap, False)

    ## To take input by clicking.
    #startPoint = (0, 10)
    #goalPoint = (199, 205)
    #print("Click start point")
    #startPoint = inputClickedPoint(occMap)
    #print("Click end point")
    #goalPoint = inputClickedPoint(occMap)

    print(startPoint, goalPoint)
    assert(gridEnv.isValidPoint(startPoint))
    assert(gridEnv.isValidPoint(goalPoint))

    startNode = Node(gridEnv.getIdFromPoint(startPoint))
    startNode.setParent(None)
    goalNode = Node(gridEnv.getIdFromPoint(goalPoint))
    gridEnv.addNode(goalNode)

    # Choose your planner.
    planner = Astar(gridEnv, inflation=1)

    # Plan!
    planFound = planner.plan(startNode, goalNode, viz=viz)

    path = []
    if planFound:
        print("Planning successful")
        # Retrieve the path.
        currNode = gridEnv.graph[goalNode.getNodeId()]
        while(currNode.getNodeId() != startNode.getNodeId()):
            path.append(currNode)
            currNode = currNode.getParent()
        # Reverse the list.
        path = path[::-1]
        print("Cost of solution is %f"%path[-1].g)

        pathPoints = []
        for node in path:
            pathPoints.append(gridEnv.getPointFromId(node.getNodeId()))

        viz.joinPointsInOrder(pathPoints, thickness=2)
        viz.displayImage()

        #Save stats.
        saveStats(planner.getPlanStats)


main()

