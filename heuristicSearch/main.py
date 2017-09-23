from astar import Astar
from env import GridEnvironment
from node import Node
from occupancyGrid import OccupancyGrid
from visualizer import ImageVisualizer
import matplotlib.pyplot as plt
import pickle

import cv2 as cv
import sys
import pickle


clickedR, clickedC = -1, -1
def inputClickedPoint(image):
    def clickCallback(event, x, y, flags, param):
        global clickedR, clickedC
        if event == cv.EVENT_LBUTTONDOWN:
            clickedC, clickedR = (x, y)

    cv.namedWindow("image")
    cv.setMouseCallback("image", clickCallback)
    cv.imshow("image", image)
    cv.waitKey(0)
    cv.destroyAllWindows()
    return (clickedR, clickedC)

def pointToRC(point):
    pointAsRC = (point[1], point[0])
    return pointAsRC

def plotStuff(planHValues, planTimePerState, stateHValues, planNodeIds=None,
        stateNodeIds=None):
    plt.figure(1)
    plt.subplot(311)
    if planNodeIds is None:
        plt.plot(planHValues)
    else:
        plt.plot(planNodeIds, planHValues)

    plt.subplot(312)
    if planNodeIds is None:
        plt.plot(planTimePerState)
    else:
        plt.plot(planNodeIds, planTimePerState)

    plt.subplot(313)
    if stateNodeIds is None:
        plt.plot(stateHValues)
    else:
        plt.plot(stateNodeIds, stateHValues)
    plt.show()

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

    occGrid = OccupancyGrid()
    occMap = occGrid.getMapFromImage(image)
    print(occMap.shape)

    useIslands = 0

    if not useIslands:
        gridEnv = GridEnvironment(occMap, occMap.shape[0], occMap.shape[1])
        gridEnv.setHeuristic(0)

    viz = ImageVisualizer(occMap)
    islands = pickle.load( open( "islands_4.pkl", "rb" ) )
    startPoint, goalPoint = pickle.load( open( "start_goal.pkl", "rb") )

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

       # plotStuff(planHValues, planTimePerState, stateHValues, planNodeIds,
       #           stateNodeIds)
        plotStuff(planHValues, planTimePerState, stateHValues)

    pathPoints = []
    for node in path:
        #print(node.getNodeId())
        pathPoints.append(gridEnv.getPointFromId(node.getNodeId()))

    viz.displayImage()
    viz.joinPointsInOrder(pathPoints, thickness=5)
    viz.displayImage()

main()

