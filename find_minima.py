from analysis.peak_finder import PeakFinder

import sys

def main():
    folder = sys.argv[1]
    peakFinder = PeakFinder()
    peakFinder.findPeaks(folder)
main()

