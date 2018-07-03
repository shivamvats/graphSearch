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

def constructIslandRegions(env, islandPoints, radii, inflation=10):
    """Construct island regions using saved island coordinates and region
    specifications."""
    islandRegions = []
    for radius, islandPoint in zip(radii, islandPoints):
        node = Node(env.getIdFromPoint(islandPoint))
        circularRegion = CircularRegion(islandPoint, radius)
        islandRegion = IslandRegion(node, circularRegion)
        islandRegion.inflation = inflation
        islandRegions.append(islandRegion)
    return islandRegions

def makePrettyViz(viz, islandRegions, startPoint, goalPoint):
    for i, region in enumerate(islandRegions):
        viz.drawCircle(region.region.center, region.region.radius)
        viz.drawCircle(region.region.center, 3, thickness=-1)
        viz.writeText("I%d"%(i+1), region.region.center[::-1], size=.8)

    viz.drawCircle(startPoint, 5, thickness=-1)
    viz.writeText("start", startPoint[::-1])
    viz.drawCircle(goalPoint, 5, thickness=-1)
    viz.writeText("goal", goalPoint[::-1])
    viz.saveImage("planning_env.jpg")

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
    islandsFile = folder + "/islands.pkl"
    islandPoints = pickle.load(open(islandsFile, "rb"))
    radii = [70 for island in islandPoints]
    islandRegions = constructIslandRegions(gridEnv, islandPoints, radii=radii,
            inflation=1.2)
    gridEnv.islandRegions = islandRegions

    # For visualization.
    viz = ImageVisualizer(occMap, False)
    makePrettyViz(viz, islandRegions, startPoint, goalPoint)

    startNode = Node(gridEnv.getIdFromPoint(startPoint))
    startNode.setParent(None)
    goalNode = Node(gridEnv.getIdFromPoint(goalPoint))
    gridEnv.addNode(goalNode.getNodeId())

    # Choose your planner.
    planner = MultiIslandAstar(gridEnv, inflation=1.2)

    # Plan!
    planFound = planner.plan(startNode, goalNode, viz=viz)

    path = []
    if planFound:
        print("Planning successful")
        # Retrieve the path.
        currNode = gridEnv.graph[goalNode.getNodeId()]
        print("Solution cost: %f"%currNode.gValue())
        while(currNode != startNode):
            path.append(currNode)
            currNode = currNode.getParent()
        # Reverse the list.
        path = path[::-1]

    pathPoints = []
    for node in path:
        pathPoints.append(gridEnv.getPointFromId(node.getNodeId()))

    #viz.joinPointsInOrder(pathPoints, thickness=2)
    viz.displayImage()

main()

