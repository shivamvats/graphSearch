from heuristicSearch.planners.astar import *
import time

class IslandAstar(Astar):
    """Vanilla island search as proposed in 'P. Chakrabarti, S.Ghose and S.
    Desarkar, "Heuristic Search through islands."
    """
    def __init__(self, env):
        super(IslandAstar, self).__init__(env)

    def plan(self, startNode, goalNode, viz=None):
        self.startNode = startNode
        self.goalNode = goalNode
        self.viz = viz

        self.startNode.setG(0)
        heuristicCost = self.env.heuristic(startNode, goalNode)
        startNode.setH(heuristicCost)

        openQ = Q.PriorityQueue()
        # Change this to a dictionary. List is too slow.
        closed = []

        openQ.put((startNode.getH() + startNode.getG(), startNode))

        currNode = startNode
        while(not openQ.empty() and currNode != self.goalNode):
            priority, currNode = openQ.get()
            if currNode in closed:
                continue
            closed.append(currNode)

            #if viz is not None:
            self.viz.markPoint(self.env.getPointFromId(currNode.getNodeId()), 0)
            self.viz.displayImage(1)

            children, edgeCosts, _ = \
            self.env.getChildrenWithIslandsAndCosts(currNode)
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
                    openQ.put((child.getG() + 2*child.getH(), child))

        print("Nodes expaneded", len(closed))
        if currNode.getNodeId() == self.goalNode.getNodeId():
            return 1
        else:
            return 0
