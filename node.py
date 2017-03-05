
class Node():
    """Base class representing an abstract Node. It contains the bare minimum
    information required for heuristic search."""
    def __init__(self, nodeId):
        self.g = float("inf")
        self.f = float("inf")
        self.nodeId = nodeId

    def setG(self, newG):
        self.g = newG

    def setParent(self, parent):
        self.parent = parent

    def getChildren(self):
        raise NotImplementedError





    # Where to put environment details?
    # Are things like the children function and heuristic properties of the node
    # or of the environment?
    # i.e Should they go outside the Node class and into a separate Environment
    # function?



