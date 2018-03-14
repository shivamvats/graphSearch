import cv2 as cv
import numpy as np
import copy

class Visualizer(object):
    def __init__(self):
        pass

class ImageVisualizer(Visualizer):
    def __init__(self, backgroundImage, incrementalDisplay=False):
        super(ImageVisualizer, self).__init__()
        self.backgroundImage = backgroundImage
        self.plottedImage = copy.copy(self.backgroundImage)
        self.incrementalDisplay = incrementalDisplay

    def resetImage( self ):
        self.plottedImage = copy.copy(self.backgroundImage)

    def drawLine(self, start, end, thickness=5, color=0):
        # Point follows convention (x, y) while the image matrix (numpy) follows
        # (r, c).
        cv.line(self.plottedImage, (start[1], start[0]), (end[1], end[0]), (100, 100, 100), thickness)

    def joinPointsInOrder(self, points, thickness=5, color=0):
        start = points[0]
        end = points[1]
        for i, point in enumerate(points[1:]):
            self.drawLine(start, end, thickness, color)
            start = end
            end = points[i]

    def markPoint(self, point, color):
        # Following (r, c) convention.
        self.plottedImage[point[0], point[1]] = color

    def markPoints(self, points, color=0):
        # Following (r, c) convention.
        points = np.array(points)#.flatten()
        self.plottedImage[points[:,0], points[:,1]] = color

    def drawCircle(self, center, radius, color=(50,50,50), thickness=1):
        #self.plottedImage = cv.circle(self.plottedImage, (center[1], center[0]), radius, (50, 50, 50))
        cv.circle(self.plottedImage, (center[1], center[0]), radius,
                color=color, thickness=thickness)

    def displayImage(self, waitTime=0):
        cv.imshow("Image", self.plottedImage)
        cv.waitKey(waitTime)
        #cv.destroyAllWindows()

    def saveImage(self, fileName):
        cv.imwrite(fileName, self.plottedImage)

    def writeText(self, text, position, size=1, font=cv.FONT_HERSHEY_SIMPLEX):
        cv.putText(self.plottedImage, text, position, font,
                size, (55,55,55), 2)




