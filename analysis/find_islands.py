import sys
import pickle

from heuristicSearch.utils import euclideanDistance
from heuristicSearch.save_points import savePoints

def findClusters( peaks, thresh ):
    """Since the peaks are in sequence, this method follows a very simplistic
    approach. For each peak it checks its distance from the previous peak. If
    it is less than threshold, it clusters that peak with the previous one.
    
    Note that in each of the clusters, input order is maintained."""
    clusters, cluster = [], []
    cluster.append(peaks[0])
    for peak in peaks[1:]:
        if euclideanDistance( cluster[-1], peak ) < thresh:
            cluster.append( peak )
        else:
            clusters.append(cluster)
            cluster = [peak]
    clusters.append( cluster )
    print( clusters )
    return clusters

def main():
    """Cluster both the heuristic and time peaks and then find out the
    heuristic cluster that belongs or corresponds to each time cluster.

    Let a time peak be T and a heuristic peak be H. Further call a time cluster
    TC and a heuristic cluster HC.

    Proposition: The number of  TC >= HC.
    Reasoning: A HC implies that consecutive states on the final  path feature
    a heuristic peak (rise in h cost. This is only possible in suboptimal
    search). The presence of a heuristic peak mandates the presence of a TC as
    extraneous states will need to be expanded.

    On the other hand, a TC need not imply an HC as either the extraneous
    states do not feature a heuristic peak (as in triangle_and_local_min2) or
    none of the extraneous states featuring a heuristic peak lies on the final
    path.
    
    Algorithm
    ---------
    
    1. Cluster T's and H's.
    2. Get the relevant clusters TC1...TCk and HC1...HCl.
    3. For each TC if there is an HC close by, mark it as its exit cluster.
    
    Note: If using a set of nodes as islands, the order in which they need to
    be traversed is also important. For this, the states must be saved in the
    correct order."""
    folder = sys.argv[1]
    timePeaksFile = folder + "/time_peaks.pkl"
    heuristicPeaksFile = folder + "/heuristic_peaks.pkl"

    timePeaks = pickle.load( open(timePeaksFile, "rb") )
    heuristicPeaks = pickle.load( open(heuristicPeaksFile, "rb") )

    print(timePeaks)
    print(heuristicPeaks)

    THRESH = 40

    # Time Clusters
    timeClusters = findClusters( timePeaks, THRESH )

    # Heuristic Clusters
    heuristicClusters = findClusters( heuristicPeaks, THRESH )

    timeHeuristicClusters = timeClusters
    for i, cluster in enumerate(timeClusters):
        lastPeak = cluster[-1]
        for j, hCluster in enumerate(heuristicClusters):
            flag = -1
            for peak in hCluster:
                if euclideanDistance( lastPeak, peak ) < THRESH:
                    flag = j
                    break
            if not flag == -1:
                timeHeuristicClusters[i] = timeHeuristicClusters[i] + hCluster
                del heuristicClusters[j]
                break
    for peak in timeHeuristicClusters:
        print peak
    savePoints( timeHeuristicClusters, folder + "/islands.pkl" )

main()

