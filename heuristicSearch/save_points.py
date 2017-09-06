import pickle
import sys

from utils import *
from occupancyGrid import OccupancyGrid

def inputPoints( numPoints, occMap ):
    points = []
    for i in range(numPoints):
        print("Click on an point")
        point = inputClickedPoint(occMap)
        points.append(point)
    return points

def savePoints(points, fileName):
    pickle.dump( points, open(fileName, "wb") )

def inputSavePoints():
    """Function to input and pickle points.
    Takes two commands line arguments: number of points and file to save them
    in."""
    numPoints = int( sys.argv[1] )
    fileName = sys.argv[2]

    occGrid = OccupancyGrid()
    occMap = occGrid.getMapFromImage("../data/complex_maze.png")

    points = inputPoints( numPoints, occMap )
    savePoints( points, fileName )

if __name__ == "__main__":
    inputSavePoints()
