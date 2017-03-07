from node import Node 

class Environment(object):
    def __init__(self, envMap):
        self.envMap = envMap

class GridEnvironment(Environment):
    def __init__(self, envMap, rows, cols):
        super(GridEnvironment, self).__init__(envMap)
        self.rows = rows
        self.cols = cols
        self.graph = {}

    def getNeighbours(self, row, col):
        """Returns 8 connected neighbours from the grid. Does validity check f
        the neighbours"""
        neighbours = []
        edgeCosts = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if(not(i == 0 and j == 0)):
                    x = row + i
                    y = col + j
                    if((x >= 0) and (x < self.rows) and (y >= 0) and (y <
                        self.cols)):
                        neighbours.append((x, y))
                        if(i == 0 or j == 0):
                            edgeCosts.append(1)
                        else:
                            edgeCosts.append(2)
        #print(neighbours)
        return (neighbours, edgeCosts)

    def getChildrenAndCosts(self, node):
        if(not self.graph.has_key(node.getNodeId())):
            self.graph[node.getNodeId()] = node
        point = self.getPointFromId(node.getNodeId())

        children, edgeCosts = self.getNeighbours(point[0], point[1])
        childrenNodes = []
        for child in children:
            nodeId = self.getIdFromPoint(child)

            if(self.graph.has_key(nodeId)):
                childrenNodes.append(self.graph[nodeId])
            else:
                node = Node(nodeId)
                self.graph[nodeId] = node
                childrenNodes.append(node)

        # XXX To do: Check if the children are free nodes.
        return (childrenNodes, edgeCosts)

    def getIdFromPoint(self, gridPoint):
        return gridPoint[0]*self.rows + gridPoint[1]

    def getPointFromId(self, Id):
        return (Id//self.rows, Id%self.rows)

