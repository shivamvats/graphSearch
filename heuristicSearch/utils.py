import cv2 as cv
import matplotlib.pyplot as plt

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
    ax = plt.subplot(211)
    ax.set_title("Heuristic change per state")
    #ax.set_xlabel("Nodes on path")
    ax.set_ylabel(r"$\Delta h$")
    if planNodeIds is None:
        plt.plot(planHValues)
    else:
        plt.plot(planNodeIds, planHValues)

    ax = plt.subplot(212)
    ax.set_title("Time taken per state")
    ax.set_xlabel("Nodes on path")
    ax.set_ylabel(r"$\Delta t$")
    if planNodeIds is None:
        plt.plot(planTimePerState)
    else:
        plt.plot(planNodeIds, planTimePerState)

    #plt.subplot(313)
    #if stateNodeIds is None:
    #    plt.plot(stateHValues)
    #else:
    #    plt.plot(stateNodeIds, stateHValues)
    plt.show()

def euclideanDistance( a, b ):
    """Assumes inputs are tuples."""
    return ( ( a[0] - b[0] )**2 + ( a[1] - b[1] )**2 )**.5

