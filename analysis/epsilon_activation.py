from sympy import *

class ActivationRegionPlotter:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.h = self.euclideanHeuristic
        self.rows, self.cols = 100, 100

    def setGoal(self, goal):
        self.goal = goal 

    def setGridSize( self, rows, cols ):
        self.rows, self.cols = rows, cols

    def circle(self, (x0, y0), radius=1):
        return ( (self.x-x0)**2 + (self.y-y0)**2 )**S.Half < radius

    def euclideanHeuristic(self, currPoint, goalPoint):
        return ((currPoint[0] - goalPoint[0])**2 + (currPoint[1] -
            goalPoint[1])**2)**S.Half

    def diagonalHeuristic(self, currPoint, goalPoint):
        dr = abs(currPoint[0] - goalPoint[0])
        dc = abs(currPoint[1] - goalPoint[1])
        D = 1
        D2 = 1.5
        return D * (dr + dc) + (D2 - 2 * D) * min(dr, dc)

    def setHeuristic(heurType):
        if heurType == 0:
            self.h = self.euclideanHeuristic
        else:
            self.h = self.diagonalHeuristic

    def getActivationRegion(self, island, epsilon ):
        heurToGoal = self.h( (self.x, self.y), (self.goal) )
        heurViaIsland = self.h( (self.x, self.y), island ) + self.h( island,
                self.goal )
        return heurViaIsland < epsilon * heurToGoal

    def plotRegion( self, island, epsilon, color, show_=False ):
        region = plot_implicit( self.getActivationRegion( island, epsilon ), (self.x,
                       0, self.cols), (self.y, 0, self.cols), show=show_,
                       line_color=color )
        return region

    def plotRegions( self, islands, epsilons, colors ):
        regions = []
        for island, epsilon, color in zip( islands, epsilons, colors ):
            regions.append( self.plotRegion( island, epsilon, color) )
        return regions

    def plotPoint( self, point, radius=1, color='r' ):
        return plot_implicit( self.circle( point, radius), (self.x, 0,
            self.cols), (self.y, 0, self.rows), line_color=color, show=False)




#if __name__ == "main":
def main():
    x, y = symbols( 'x, y' )
    plotter = ActivationRegionPlotter( x, y )

    print( "Plotting" )
    plotter.setGoal( (50, 5) )
    island = (80, 50)
    islands = [ (80, 50), (20, 50) ]
    epsilons = [1.5, 1.5]
    colors = ['r', 'b']
    #plotter.plotRegion( island, 1.2 )
    regions = plotter.plotRegions( islands, epsilons, colors )
    goalPoint = plotter.plotPoint( plotter.goal )
    finalRegion = regions[0]
    for region in regions + [ goalPoint, ]:
        finalRegion.extend(region)
    finalRegion.show()


main()
