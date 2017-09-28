from astar import Astar, IslandAstar
from env import GridEnvironment, IslandGridEnvironment
from node import Node
from occupancyGrid import OccupancyGrid
from visualizer import ImageVisualizer
import matplotlib.pyplot as plt
from utils import *
from save_points import savePoints

import cv2 as cv
import sys
import pickle

class PeakFinder:
    def __init__(self):
        pass

    def getTimePerState( self, planTime ):
        planTimePerState = []
        planTimePerState.append(0)
        for i in range(1, len(planTime)):
            planTimePerState.append( planTime[i] - planTime[i-1] )
        return planTimePerState

    def getPeaks( self, nodeIds, planTime ):
        """Takes in nodeIDs and timestamps and returns the node ids of
        peaks sorted in descending order by time per state."""
        # The indices of nodeIds and planTime match.
        # Functions that operate on planTime must maintain this one-one
        # correspondence.
        planTimePerState = self.getTimePerState( planTime )

        peakNodeIds, peakTimePerState = [], []
        for i in range( 1, len(planTime) - 1 ):
            backwardDiff = planTimePerState[i] - planTimePerState[i-1]
            forwardDiff = planTimePerState[i+1] - planTimePerState[i]
            # Note: The time per state must be positive, else it it not a peak.
            if planTimePerState[i] > 0 and backwardDiff > 0 and forwardDiff < 0:
                peakNodeIds.append( nodeIds[i] )
                peakTimePerState.append( planTimePerState[i] )
        # Sort according to time per state.
        print( len(peakTimePerState) )
        sortedIds = [node for _, node in sorted( zip(peakTimePerState, peakNodeIds),
            reverse=True )]
        print( sorted(peakTimePerState, reverse=True)[:5] )
        return sortedIds

    def savePeaks( self, timePeaks, heuristicPeaks ):
        def sortPeaks( peaks ):
            # The peaks need to saved in the order in which they were expanded.
            indices = []
            for peak in peaks:
                indices.append( self.solutionPath.index( peak ) )
            sortedPeaks = [peak for _, peak in sorted( zip(indices,
                peaks) )]
            return sortedPeaks

        savePoints( map( self.gridEnv.getPointFromId, sortPeaks( timePeaks ) ),
                self.folder + "/time_peaks.pkl" )
        savePoints( map( self.gridEnv.getPointFromId, sortPeaks( heuristicPeaks ) ),
                self.folder + "/heuristic_peaks.pkl" )

    def findPeaks(self, folder):
        """Numpy array is accessed as (r, c) while a point is (x, y). The code
        follows (r, c) convention everywhere. Hence, be careful whenever using a
        point with opencv.

        Takes one command line argument: Folder that has the environment and
        config.
        """

        self.folder = folder
        image = self.folder + "/image.png"
        start_goal = folder + "/start_goal.pkl"
        startPoint, goalPoint = pickle.load( open(start_goal, "rb") )
        NUMTIMEPEAKS = 5
        NUMHEURISTICPEAKS = 3

        occGrid = OccupancyGrid()
        occMap = occGrid.getMapFromImage(image)
        print(occMap.shape)

        self.gridEnv = GridEnvironment(occMap, occMap.shape[0], occMap.shape[1])
        self.gridEnv.setHeuristic(0)

        viz = ImageVisualizer(occMap)

        #startPoint = (100, 20)
        #goalPoint = (201, 200)
        #print("Click start point")
        #startPoint = inputClickedPoint(occMap)
        #print("Click end point")
        #goalPoint = inputClickedPoint(occMap)
        print(startPoint, goalPoint)
        assert(self.gridEnv.isValidPoint(startPoint))
        assert(self.gridEnv.isValidPoint(goalPoint))

        startNode = Node(self.gridEnv.getIdFromPoint(startPoint))
        startNode.setParent(None)
        goalNode = Node(self.gridEnv.getIdFromPoint(goalPoint))
        self.gridEnv.addNode(goalNode)

        planner = Astar(self.gridEnv)
        planFound = planner.plan(startNode, goalNode, viz=viz)
        #planFound = planner.plan(startNode, goalNode)

        path = []
        planNodeIds = []
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
            self.solutionPath = planStateIds
            # Dictionary of ids and timestamps.
            planStats = planner.getPlanStats()

            # -----------------------------------
            # Finding the local minima.
            # Get the timestamps.
            stateNodeIds = planStats.keys()
            stateHValues = []
            for i in planStats.values():
                stateHValues.append(i[1])

            planHValues, planTimeStamps, planTimePerState = [], [], []

            for stateId in planStateIds:
                planNodeIds.append(stateId)
                planHValues.append(planStats[stateId][1])
                planTimeStamps.append(planStats[stateId][0])

            # Start state.
            planTimePerState = [0]
            for i in range(1, len(planTimeStamps)):
                planTimePerState.append(planTimeStamps[i] - planTimeStamps[i-1])
            planHeuristicPerState = [0]
            for i in range(1, len(planHValues)):
                planHeuristicPerState.append(max(0, planHValues[i] -
                    planHValues[i-1]) )

        # plotStuff(planHValues, planTimePerState, stateHValues, planNodeIds,
        #           stateNodeIds)
            #plotStuff(planHValues, planTimePerState, stateHValues)
            plotStuff(planHeuristicPerState, planTimePerState, stateHValues)

        # Visualize the path.
        pathPoints = []
        for node in path:
            #print(node.getNodeId())
            pathPoints.append(self.gridEnv.getPointFromId(node.getNodeId()))
        #viz.joinPointsInOrder(pathPoints, thickness=5)

        #----------------------------------------
        # Visualize the peaks
        # Extract the timestamps from values.
        pathPlanTime = [ planStats[node][0] for node in planNodeIds ]
        # Peaks has the node ids sorted according to the delta t.
        peaks = self.getPeaks( planNodeIds, pathPlanTime )
        timePeaks = peaks[:NUMTIMEPEAKS]

        # Extract the heuristic values.
        pathPlanHeuristic= [ planStats[node][1] for node in planNodeIds ]
        # Peaks has the node ids sorted according to the delta t.
        print( "Finding heuristic peaks.")
        peaks = self.getPeaks( planNodeIds, pathPlanHeuristic )
        heuristicPeaks = peaks#[:NUMHEURISTICPEAKS]
        #for peak in timePeaks:
            #print(self.gridEnv.getPointFromId(peak))

        for peak in timePeaks:
            print("Marking the minima")
            #print(self.gridEnv.getPointFromId(peak))
            viz.drawCircle(self.gridEnv.getPointFromId(peak), 5, color=(200,200,200), thickness=-1)
        for peak in heuristicPeaks:
            print("Marking the minima")
            #viz.drawCircle(self.gridEnv.getPointFromId(peak), 5, color=(150,150,150), thickness=-1)
        #---------------------------------------
        viz.displayImage()

        # Save the peaks in files.
        self.savePeaks( timePeaks, heuristicPeaks )

def main():
    folder = sys.argv[1]
    peakFinder = PeakFinder()
    peakFinder.findPeaks(folder)
main()

# XXX Need to review this
"""
def getHeuristicDropPerState( planTime ):
    planTimePerState = []
    planTimePerState.append(0)
    for i in range(1, len(planTime)):
        planTimePerState.append( planTime[i-1] - planTime[i] )
    return planTimePerState

def getHeuristicPeaks( nodeIds, planTime ):
    #Takes in nodeIDs and timestamps and returns the node ids of
    #peaks sorted in descending order by time per state.
    # The indices of nodeIds and planTime match.
    # Functions that operate on planTime must maintain this one-one
    # correspondence.
    planTimePerState = getTimePerState( planTime )

    peakNodeIds, peakTimePerState = [], []
    for i in range( 1, len(planTime) - 1 ):
        backwardDiff = planTimePerState[i] - planTimePerState[i-1]
        forwardDiff = planTimePerState[i+1] - planTimePerState[i]
        if backwardDiff < 0 and forwardDiff > 0:
            peakNodeIds.append( nodeIds[i] )
            peakTimePerState.append( planTimePerState[i] )
    # Sort according to time per state.
    print( len(peakTimePerState) )
    sortedIds = [node for _, node in sorted( zip(peakTimePerState, peakNodeIds),
        reverse=True )]
    #print( sorted(peakTimePerState, reverse=True) )
    return sortedIds
"""
