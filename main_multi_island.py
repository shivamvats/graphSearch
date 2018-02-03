from heuristicSearch.planners.multi_island_astar import MultiIslandAstar
from heuristicSearch.envs.multi_island_env import MultiIslandGridEnvironment
from heuristicSearch.graph.node import Node
from heuristicSearch.graph.island_region import IslandRegion
from heuristicSearch.graph.circular_region import CircularRegion
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

def constructIslandRegions(env, folder, radius=20, inflation=10):
    """Construct island regions using saved island coordinates and region
    specifications."""
    islandsFile = folder + "/islands.pkl"
    islandPoints = pickle.load(open(islandsFile, "rb"))
    print(islandPoints)
    islandRegions = []
    for islandPoint in islandPoints:
        node = Node(env.getIdFromPoint(islandPoint))
        circularRegion = CircularRegion(islandPoint, radius)
        islandRegion = IslandRegion(node, circularRegion)
        islandRegion.inflation = inflation
        islandRegions.append(islandRegion)
    return islandRegions

def main():
    folder = sys.argv[1]
    image = folder + "/image.png"
    start_goal = folder + "/start_goal.pkl"
    startPoint, goalPoint = pickle.load( open(start_goal, "rb") )
    # Build the planning environment.
    occGrid = OccupancyGrid()
    occMap = occGrid.getMapFromImage(image)
    gridEnv = MultiIslandGridEnvironment(occMap, occMap.shape[0], occMap.shape[1])
    gridEnv.setHeuristicType(0)
    assert(gridEnv.isValidPoint(startPoint))
    assert(gridEnv.isValidPoint(goalPoint))

    # Set up island regions.
    islandRegions = constructIslandRegions(gridEnv, folder, radius=50,
            inflation=5)
    gridEnv.islandRegions = islandRegions

    # For visualization.
    viz = ImageVisualizer(occMap, True)
    for region in islandRegions:
        viz.drawCircle(region.region.center, region.region.radius)

    startNode = Node(gridEnv.getIdFromPoint(startPoint))
    startNode.setParent(None)
    goalNode = Node(gridEnv.getIdFromPoint(goalPoint))
    gridEnv.addNode(goalNode)

    # Choose your planner.
    planner = MultiIslandAstar(gridEnv, inflation=5)

    # Plan!
    planFound = planner.plan(startNode, goalNode, viz=viz)

    path = []
    if planFound:
        print("Planning successful")
        # Retrieve the path.
        currNode = goalNode
        while(currNode != startNode):
            path.append(currNode)
            currNode = currNode.getParent()
        # Reverse the list.
        path = path[::-1]

    pathPoints = []
    for node in path:
        pathPoints.append(gridEnv.getPointFromId(node.getNodeId()))

    viz.joinPointsInOrder(pathPoints, thickness=2)
    viz.displayImage()

main()

