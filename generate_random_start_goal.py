import numpy as np
import sys
import os
import Queue as Q

from save_points import inputPoints, savePoints
from heuristicSearch.envs.occupancyGrid import OccupancyGrid
from heuristicSearch.envs.env import GridEnvironment

def generateStartGoal(occMap):
    print("Choose approximate start and goal.")
    approxStart, approxGoal = inputPoints(2, occMap)

    env = GridEnvironment(occMap, occMap.shape[0], occMap.shape[1])
    # BFS
    openStart = Q.Queue()
    openStart.put(approxStart)
    currStart = approxStart
    print(currStart)
    while not env.isValidPoint(currStart):
        currStart = openStart.get()
        nbrs, _ = env.getNeighbours(currStart[0], currStart[1])
        for pixel in nbrs:
            openStart.put(pixel)
    start = currStart

    openGoal = Q.Queue()
    openGoal.put(approxGoal)
    currGoal = approxGoal
    while not env.isValidPoint(currGoal):
        currGoal = openGoal.get()
        nbrs, _ = env.getNeighbours(currGoal[0], currGoal[1])
        for pixel in nbrs:
            openGoal.put(pixel)
    goal = currGoal
    print("Generated start-goal", (start, goal))

    return [start, goal]

def saveRandomStartGoal():
    "Assumes binary image."
    folder = sys.argv[1]
    image = folder + "/image.png"
    start_goal_file = folder + "/start_goal.pkl"
    occGrid = OccupancyGrid()
    occMap = occGrid.getMapFromImage(image)

    startGoal = generateStartGoal(occMap)
    savePoints(startGoal, start_goal_file)
    print("Start-goal saved.")

if __name__ == "__main__":
    saveRandomStartGoal()
