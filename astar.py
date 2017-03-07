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

    def plan(self, startNode, goalNode, heuristic, viz=None):
        self.startNode = startNode
        self.goalNode = goalNode

        self.startNode.setG(0)
        heuristicCost = heuristic(startNode, goalNode, self.env)
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
            
            #if viz is not None:
            viz.markPoint(self.env.getPointFromId(currNode.getNodeId()), 0)
            viz.displayImage(1)
            #print(openQ.qsize())
            #print(currNode.getH())
            
            children, edgeCosts = self.env.getChildrenAndCosts(currNode)
            for child, edgeCost in zip(children, edgeCosts):
                if child in closed:
                    continue
                
                if child.getH() == float("inf"):
                    child.setH(heuristic(child, goalNode, self.env))

                #print(currNode.getH())
                updated = self.updateG(child, currNode.getG() + edgeCost)
                if updated:
                    child.setParent(currNode)
                #XXX What if this node is already in the open list?
                openQ.put((child.getG() + child.getH(), child))

        if currNode.getNodeId() == self.goalNode.getNodeId():
            return 1
        else:
            return 0




            







