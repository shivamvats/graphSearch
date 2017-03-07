import cv2 as opencv

class OccupancyGrid(object):
    def __init__(self):
        pass

    def displayMap(self):
        opencv.imshow("image", self.image)
        opencv.waitKey(0)
        opencv.destroyAllWindows()


    def getMapFromImage(self, image):
        # Read in grayscale.
        self.image = opencv.imread(image, 0)

        # Map is anyway binary. So this is a 0-1 occupancy grid.
        return self.image



