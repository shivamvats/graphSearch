import matplotlib.pyplot as plt
import cv2 as cv
import numpy as np

from scipy.stats import norm
import random

def sampleNormal(mean=0, dev=1):
    randProb = random.random()
    sample = norm.ppf(randProb)
    return dev*sample + mean


def generateRandomImage(size=(100, 100), mean=0, dev=1):
    randomMap = np.zeros(shape=size)
    randomMap.fill(255)
    for row in range(size[0]):
        for col in range(size[1]):
            randNormalVal = (sampleNormal(mean, dev) + 1)/2
            randNormalTruncated = min(randNormalVal, 1)
            randNormalTruncated = max(randNormalTruncated, 0)
            randPixelVal = 255*randNormalTruncated
            if randPixelVal < 120:
                randomMap.itemset((row, col), 0)

    kernel = np.ones((5,5),np.uint8)
    dilation = cv.erode(randomMap,kernel,iterations = 10)
    return randomMap


def displayImage(img):
    cv.namedWindow('image', cv.WINDOW_NORMAL)
    cv.imshow('image', img)
    cv.waitKey(0)
    cv.destroyAllWindows()


def main():
    randomMap = generateRandomImage(size=(600, 600))
    displayImage(randomMap)
    cv.imwrite("rand_normal.png", randomMap)


if __name__ == "__main__":
    main()


