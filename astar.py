import Queue as Q
import time

class Astar(object):
    def __init__(self, env):
        self.env = env

    def updateG(self, node, newG):
        if(node.getG() > newG):
            node.setG(newG)
            return 1
        else:
            return 0

    def plan(self, startNode, goalNode, viz=None):
        self.startNode = startNode
        self.goalNode = goalNode

        self.startNode.setG(0)
        heuristicCost = self.env.heuristic(startNode, goalNode)
        startNode.setH(heuristicCost)

        openQ = Q.PriorityQueue()
        closed = []

        openQ.put((startNode.getH() + startNode.getG(), startNode))

        currNode = startNode
        while(not openQ.empty() and currNode != self.goalNode):
            priority, currNode = openQ.get()
            if currNode in closed:
                continue
            closed.append(currNode)
            #print(currNode.getG()+ currNode.getH())
            if currNode in self.env.activatedIslandNodes:
                print("Island node expanded", "g = ", currNode.getG())
            
            #if viz is not None:
            viz.markPoint(self.env.getPointFromId(currNode.getNodeId()), 0)
            viz.displayImage(1)
            
            children, edgeCosts, islandFound = self.env.getChildrenAndCosts(currNode)
            if islandFound:
                # XXX Assumes only one island child and it is the last element
                # in the list.
                islandChild = children[-1]
                #print("Island node", self.env.getPointFromId(islandChild.getNodeId()))
                #print("Cost is ", edgeCosts[-1])
                if currNode in self.env.activatedIslandNodes:
                    pass
                elif self.env.ancestoryContainsNode(currNode, islandChild,
                        self.env.getIslandThresh()):
                    #print(currNode.getNodeId(), islandChild.getNodeId())
                    pass
                elif islandChild in self.env.activatedIslandNodes:
                    continue
                self.env.activateIslandNode(islandChild)
            for child, edgeCost in zip(children, edgeCosts):
                if child in closed:
                    continue
                
                if child.getH() == float("inf"):
                    child.setH(self.env.heuristic(child, goalNode))

                #print(currNode.getH())
                updated = self.updateG(child, currNode.getG() + edgeCost)
                if updated:
                    child.setParent(currNode)
                    #XXX What if this node is already in the open list?
                    openQ.put((child.getG() + child.getH(), child))

        print("Nodes expaneded", len(closed))
        if currNode.getNodeId() == self.goalNode.getNodeId():
            return 1
        else:
            return 0




            







