from heuristicSearch.planners.dynamic_mha import DynamicMHAstar
from heuristicSearch.envs.island_cluster_env import IslandClusterGridEnvironment
from heuristicSearch.graph.node import Node
from heuristicSearch.envs.occupancyGrid import OccupancyGrid
from heuristicSearch.utils.visualizer import ImageVisualizer
from heuristicSearch.utils.utils import *

import matplotlib.pyplot as plt
import cv2 as cv
import sys
import pickle

def main():
    """Numpy array is accessed as (r, c) while a point is (x, y). The code
    follows (r, c) convention everywhere. Hence, be careful whenever using a
    point with opencv.

    Takes one command line argument: Folder that has the environment and
    config.
    """

    folder = sys.argv[1]
    image = folder + "/image.png"
    start_goal = folder + "/start_goal.pkl"
    islands = folder + "/islands.pkl"
    startPoint, goalPoint = pickle.load( open(start_goal, "rb") )
    islandClusters = pickle.load( open(islands, "rb") )

    # Ignore the states in between.
    # Directly more towards the exit state.
    activationCenters, exitStates = [], []
    for cluster in islandClusters:
        activationCenters.append(cluster[0])
        exitStates.append(cluster[-1])

    print(islandClusters)

    occGrid = OccupancyGrid()
    occMap = occGrid.getMapFromImage(image)
    print(occMap.shape)

    gridEnv = IslandClusterGridEnvironment(occMap, occMap.shape[0],
            occMap.shape[1], activationCenters, exitStates )
    gridEnv.setHeuristic(0)

    viz = ImageVisualizer(occMap)

    #startPoint = (100, 20)
    #goalPoint = (201, 200)
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
    gridEnv.ISLANDTHRESH = 40

    planner = DynamicMHAstar(gridEnv, 2, 10)
    planner.DEBUG = 0
    planFound = planner.plan(startNode, goalNode, viz=viz)
    #planFound = planner.plan(startNode, goalNode)

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

        planStateIds = map(lambda node : node.getNodeId(), path)
        planStats = planner.getPlanStats()

        # -----------------------------------
        # Finding the local minima.
        # Get the timestamps.
        stateNodeIds = planStats.keys()
        stateHValues = []
        for i in planStats.values():
            stateHValues.append(i[1])

        planHValues, planTimeStamps, planTimePerState = [], [], []

        planNodeIds = []
        for stateId in planStateIds:
            planNodeIds.append(stateId)
            planHValues.append(planStats[stateId][1])
            planTimeStamps.append(planStats[stateId][0])

        # Start state.
        planTimePerState = [0]
        for i in range(1, len(planTimeStamps)):
            planTimePerState.append(planTimeStamps[i] - planTimeStamps[i-1])

        #plotStuff(planHValues, planTimePerState, stateHValues, planNodeIds,
                  #stateNodeIds)
        plotStuff(planHValues, planTimePerState, stateHValues)

    pathPoints = []
    for node in path:
        #print(node.getNodeId())
        pathPoints.append(gridEnv.getPointFromId(node.getNodeId()))

    viz.displayImage()
    viz.joinPointsInOrder(pathPoints, thickness=5)
    viz.displayImage()

main()

