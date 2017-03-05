
class Environment():
    def __init__(self):
        pass

class GridEnvironment(Environment):
    def __init__(self, xdim, ydim):
        self.xdim = xdim
        self.ydim = ydim

    def getNeighbours(self, xcood, ycood):
        """Returns 8 connected neighbours from the grid. Does validity check f
        the neighbours"""
        neighbours = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if(not(i == 0 and j == 0))
                    x = xcood + i
                    y = ycood + j
                    if((x >= 0) and (x < self.xdim) and (y >= 0) and (y <
                        self.ydim)):
                        neighbours.append((x, y))
        return neighbours


