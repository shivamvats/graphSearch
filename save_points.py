from heuristicSearch.utils.utils import *

def inputSavePoints():
    """Function to input and pickle points.
    Takes three commands line arguments: Map to be used, file to be saved in
    and number of points.
    """
    image = sys.argv[1]
    fileName = sys.argv[2]
    numPoints = int( sys.argv[3] )

    occGrid = OccupancyGrid()
    occMap = occGrid.getMapFromImage(image)

    points = inputPoints( numPoints, occMap )
    savePoints( points, fileName )

if __name__ == "__main__":
    inputSavePoints()

