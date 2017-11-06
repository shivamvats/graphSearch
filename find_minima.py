from analysis.peak_finder import PeakFinder

def main():
    folder = sys.argv[1]
    peakFinder = PeakFinder()
    peakFinder.findPeaks(folder)
main()

