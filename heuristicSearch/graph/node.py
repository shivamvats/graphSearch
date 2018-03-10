class Node():
    """Base class representing an abstract Node. It contains the bare minimum
    information required for heuristic search."""
    def __init__(self, nodeId, viaIsland=False):
        self._g = float("inf")
        self._h = float("inf")
        self.nodeId = nodeId

        # Island search stuff
        self.h1 = 0
        self.viaIsland = viaIsland
        #self.hasDummyG = False

    @property
    def g(self):
        return self._g

    @property
    def h(self):
        return self._h

    @g.setter
    def g(self, newG):
        self._g = newG

    @h.setter
    def h(self, newH):
        self._h = newH

    def setParent(self, parent):
        self.parent = parent

    def getH1(self):
        return self.h1

    def setH1(self, val):
        self.h1 = val

    def getGvalue(self):
        return self.g + self.h1

    def setViaIsland(self, viaIsland):
        self.viaIsland = viaIsland

    def isViaIsland(self):
        return self.viaIsland

    def getNodeId(self):
        return self.nodeId

    def getParent(self):
        return self.parent

    def setHasDummyG(self, value):
        self.hasDummyG = value

    def checkDummyG(self):
        return self.hasDummyG


class SkipEdgesNode(Node):
    def __init__(self, nodeId, hasDummyG=False):
        super(IslandNode, self).__init__(nodeId)
        self.dummyG = float("inf")
        if hasDummyG:
            self.hasDummyG = True
        else:
            self.hasDummyG = False

class NIslandNode(Node):
    def __init__(self, nodeId, viaIsland=False):
        super(IslandNode, self).__init__(nodeId)

        self.h1 = 0
        self.viaIsland = viaIsland

    def getH1(self):
        return self.h1

    def setH1(self, val):
        self.h1 = val

    def getGvalue(self):
        return self.g + self.h1

    def setViaIsland(self, viaIsland):
        self.viaIsland = viaIsland

    def isViaIsland(self):
        return self.viaIsland


    #def getG(self, newG):


    # Where to put environment details?
    # Are things like the children function and heuristic properties of the node
    # or of the environment?
    # i.e Should they go outside the Node class and into a separate Environment
    # function?



