
class Node():
    """Base class representing an abstract Node. It contains the bare minimum
    information required for heuristic search."""
    def __init__(self, nodeId):
        self.g = float("inf")
        self.h = float("inf")
        self.nodeId = nodeId

        # For the n-island heuristic in local minima algo.
        self.history = []

        self.hasDummyG = False

    def setG(self, newG):
        self.g = newG

    def setH(self, newH):
        self.h = newH

    def setParent(self, parent):
        self.parent = parent

    def getG(self):
        return self.g

    def getH(self):
        return self.h

    def getNodeId(self):
        return self.nodeId

    def getParent(self):
        return self.parent

    def isEqualTo(node):
        return self.getNodeId() == node.getNodeId

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

class IslandNode(Node):
    def __init__(self, nodeId, viaIsland=False):
        super(IslandNode, self).__init__(nodeId)

        self.viaIsland = viaIsland

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



