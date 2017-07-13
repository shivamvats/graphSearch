from astar import Astar, IslandAstar
from env import GridEnvironment, IslandGridEnvironment
from node import Node
from occupancyGrid import OccupancyGrid
from visualizer import ImageVisualizer

import cv2 as cv


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


def main():
    """Numpy array is accessed as (r, c) while a point is (x, y). The code
    follows (r, c) convention everywhere. Hence, be careful whenever using a
    point with opencv."""
    occGrid = OccupancyGrid()
    occMap = occGrid.getMapFromImage("../data/testMap.png")
    print(occMap.shape)

    useIslands = 0

    if not useIslands:
        gridEnv = GridEnvironment(occMap, occMap.shape[0], occMap.shape[1])
        gridEnv.setHeuristic(0)

    viz = ImageVisualizer(occMap)

    ###
    # Island related stuff
    ###
    #"""
    if useIslands:
        #islands = [(140, 100),]
        islands = []
        numIslands = 1
        for i in range(numIslands):
            print("Click on an island")
            island = inputClickedPoint(occMap)
            islands.append(island)
        gridEnv = IslandGridEnvironment(occMap, occMap.shape[0], occMap.shape[1],
                islands)
        gridEnv.setHeuristic(1)
        gridEnv.setIslandThresh(80)
        for island in gridEnv.getIslandNodes():
            viz.drawCircle(gridEnv.getPointFromId(island.getNodeId()), 80)
            viz.displayImage(1)
        cv.destroyAllWindows()

    #"""
    ###

    #startPoint = (100, 20)
    #goalPoint = (201, 200)
    print("Click start point")
    startPoint = inputClickedPoint(occMap)
    print("Click end point")
    goalPoint = inputClickedPoint(occMap)
    print(startPoint, goalPoint)
    assert(gridEnv.isValidPoint(startPoint))
    assert(gridEnv.isValidPoint(goalPoint))

    startNode = Node(gridEnv.getIdFromPoint(startPoint))
    startNode.setParent(None)
    goalNode = Node(gridEnv.getIdFromPoint(goalPoint))
    gridEnv.addNode(goalNode)

    if useIslands:
        planner = IslandAstar(gridEnv)
    else:
        planner = Astar(gridEnv)
    planFound = planner.plan(startNode, goalNode, viz=viz)
    #planFound = planner.plan(startNode, goalNode)

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

