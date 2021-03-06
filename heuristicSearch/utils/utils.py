import cv2 as cv
import matplotlib.pyplot as plt
import pickle
import sys

from heuristicSearch.envs.occupancy_grid import OccupancyGrid

clickedR, clickedC = -1, -1
def inputClickedPoint(image):
    """Displays an environment and tells you the row and column number of a
    clicked point in the map."""
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

def euclideanDistance( a, b ):
    """Assumes inputs are tuples."""
    return ( ( a[0] - b[0] )**2 + ( a[1] - b[1] )**2 )**.5

def inputPoints( numPoints, occMap ):
    points = []
    for i in range(numPoints):
        print("Click on an point")
        point = inputClickedPoint(occMap)
        points.append(point)
    return points

def savePoints(points, fileName):
    pickle.dump( points, open(fileName, "wb") )
