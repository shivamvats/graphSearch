from astar import Astar, IslandAstar
from env import GridEnvironment, IslandGridEnvironment
from node import Node
from occupancyGrid import OccupancyGrid
from visualizer import ImageVisualizer
import matplotlib.pyplot as plt
from utils import *

import cv2 as cv
import sys
import pickle

def getTimePerState( planTime ):
    planTimePerState = []
    planTimePerState.append(0)
    for i in range(1, len(planTime)):
        planTimePerState.append( planTime[i] - planTime[i-1] )
    return planTimePerState

def getPeaks( nodeIds, planTime ):
    """Takes in nodeIDs and timestamps and returns the node ids of
    peaks sorted in descending order by time per state."""
    # The indices of nodeIds and planTime match.
    # Functions that operate on planTime must maintain this one-one
    # correspondence.
    planTimePerState = getTimePerState( planTime )

    peakNodeIds, peakTimePerState = [], []
    for i in range( 1, len(planTime) - 1 ):
        backwardDiff = planTimePerState[i] - planTimePerState[i-1]
        forwardDiff = planTimePerState[i+1] - planTimePerState[i]
        if backwardDiff > 0 and forwardDiff < 0:
            peakNodeIds.append( nodeIds[i] )
            peakTimePerState.append( planTimePerState[i] )
    # Sort according to time per state.
    print( len(peakTimePerState) )
    sortedIds = [node for _, node in sorted( zip(peakTimePerState, peakNodeIds),
        reverse=True )]
    #print( sorted(peakTimePerState, reverse=True) )
    return sortedIds

# XXX Ned to review this
"""
def getHeuristicDropPerState( planTime ):
    planTimePerState = []
    planTimePerState.append(0)
    for i in range(1, len(planTime)):
        planTimePerState.append( planTime[i-1] - planTime[i] )
    return planTimePerState

def getHeuristicPeaks( nodeIds, planTime ):
    #Takes in nodeIDs and timestamps and returns the node ids of
    #peaks sorted in descending order by time per state.
    # The indices of nodeIds and planTime match.
    # Functions that operate on planTime must maintain this one-one
    # correspondence.
    planTimePerState = getTimePerState( planTime )

    peakNodeIds, peakTimePerState = [], []
    for i in range( 1, len(planTime) - 1 ):
        backwardDiff = planTimePerState[i] - planTimePerState[i-1]
        forwardDiff = planTimePerState[i+1] - planTimePerState[i]
        if backwardDiff < 0 and forwardDiff > 0:
            peakNodeIds.append( nodeIds[i] )
            peakTimePerState.append( planTimePerState[i] )
    # Sort according to time per state.
    print( len(peakTimePerState) )
    sortedIds = [node for _, node in sorted( zip(peakTimePerState, peakNodeIds),
        reverse=True )]
    #print( sorted(peakTimePerState, reverse=True) )
    return sortedIds
"""

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
    startPoint, goalPoint = pickle.load( open(start_goal, "rb") )
    NUMTIMEPEAKS = 10
    NUMHEURISTICPEAKS = 3

    occGrid = OccupancyGrid()
    occMap = occGrid.getMapFromImage(image)
    print(occMap.shape)

    gridEnv = GridEnvironment(occMap, occMap.shape[0], occMap.shape[1])
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

    planner = Astar(gridEnv)
    planFound = planner.plan(startNode, goalNode, viz=viz)
    #planFound = planner.plan(startNode, goalNode)

    path = []
    planNodeIds = []
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
        # Dictionary of ids and timestamps.
        planStats = planner.getPlanStats()

        # -----------------------------------
        # Finding the local minima.
        # Get the timestamps.
        stateNodeIds = planStats.keys()
        stateHValues = []
        for i in planStats.values():
            stateHValues.append(i[1])

        planHValues, planTimeStamps, planTimePerState = [], [], []

        for stateId in planStateIds:
            planNodeIds.append(stateId)
            planHValues.append(planStats[stateId][1])
            planTimeStamps.append(planStats[stateId][0])

        # Start state.
        planTimePerState = [0]
        for i in range(1, len(planTimeStamps)):
            planTimePerState.append(planTimeStamps[i] - planTimeStamps[i-1])
        planHeuristicPerState = [0]
        for i in range(1, len(planHValues)):
            planHeuristicPerState.append(max(0, planHValues[i] -
                planHValues[i-1]) )


       # plotStuff(planHValues, planTimePerState, stateHValues, planNodeIds,
       #           stateNodeIds)
        #plotStuff(planHValues, planTimePerState, stateHValues)
        plotStuff(planHeuristicPerState, planTimePerState, stateHValues)

    # Visualize the path.
    pathPoints = []
    for node in path:
        #print(node.getNodeId())
        pathPoints.append(gridEnv.getPointFromId(node.getNodeId()))
    viz.joinPointsInOrder(pathPoints, thickness=5)

    #----------------------------------------
    # Visualize the peaks
    # Extract the timestamps from values.
    pathPlanTime = [ planStats[node][0] for node in planNodeIds ]
    # Peaks has the node ids sorted according to the delta t.
    peaks = getPeaks( planNodeIds, pathPlanTime )
    timePeaks = peaks[:NUMTIMEPEAKS]

    # Extract the heuristic values.
    pathPlanHeuristic= [ planStats[node][1] for node in planNodeIds ]
    # Peaks has the node ids sorted according to the delta t.
    peaks = getPeaks( planNodeIds, pathPlanHeuristic )
    heuristicPeaks = peaks[:NUMHEURISTICPEAKS]

    #localMinima = planTimePerState[:NUMPEAKS]
    for peak in timePeaks:
        print("Marking the minima")
        viz.drawCircle(gridEnv.getPointFromId(peak), 5, color=(200,200,200), thickness=-1)
    for peak in heuristicPeaks:
        print("Marking the minima")
        viz.drawCircle(gridEnv.getPointFromId(peak), 5, color=(150,150,150), thickness=-1)
    #---------------------------------------
    viz.displayImage()

main()

