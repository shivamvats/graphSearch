from astar import *

class IslandAstar(Astar):
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

class DummyEdgeAstar(Astar):
    def __init__(self, env):
        super(DummyEdgeAstar, self).__init__(env)

    def updateDummyG(self, node, newG):
        node.setHasDummyG(False)
        if(node.getG() > newG or node.checkDummyG()):
            node.setG(newG)
            return 1
        else:
            return 0

    def updateGvalue(self, node, newG, newH1):
        if(node.getGvalue() > newG + newH1):
            node.setG(newG)
            node.setH1(newH1)
            return 1
        else:
            return 0

    def _expand1(self, node):
        """Usual expand method that doesn't worry about islands."""
        #if viz is not None:
        self.viz.markPoint(self.env.getPointFromId(node.getNodeId()), 0)
        self.viz.displayImage(1)

        succs, edgeCosts = self.env.getChildrenAndCosts( node )

        for succ, cost in zip( succs, edgeCosts ):
            updated = self.updateGvalue( succ, node.getG() + cost, node.getH1() )
            if updated:
                succ.setParent( node )
                self.OPEN.put( (self.env.fValue(succ), succ) )


    def _expand2(self, node, island=None):
        """Modified expand method that also allows for dummy edges to nearby
        islands."""
        #if node in self.env.activeIslandNodes:
        #    print("Island node expanded", "g = ", node.getG())

        #if viz is not None:
        self.viz.markPoint(self.env.getPointFromId(node.getNodeId()), 0)
        self.viz.displayImage(1)

        succs, dummySuccs, edgeCosts, dummyCosts = \
            self.env.getChildrenIslandsAndCosts(node)

        def _insertInOpen( node, succ, island ):
            succ.setParent( node )
            self.OPEN.put( (self.env.fValue( succ, island), succ) )

        for succ, cost in zip(succs, edgeCosts):
            updated = self.updateGvalue(succ, node.getG() + cost, node.getH1())
            if updated:
                _insertInOpen( node, succ, island )

        for succ, cost in zip( dummySuccs, dummyCosts ):
            updated = self.updateGvalue( succ, node.getG(), node.getH1() + cost )
            if updated:
                #print( "Updated node: %d, cost %f", (node.getNodeId(),
                    #self.env.fValue(succ)) )
                _insertInOpen( node, succ, island )


    def plan(self, startNode, goalNode, viz=None):
        self.startNode = startNode
        self.goalNode = goalNode
        self.viz = viz

        self.startNode.setG(0)
        heuristicCost = self.env.heuristic(startNode, goalNode)
        startNode.setH(heuristicCost)

        self.OPEN = Q.PriorityQueue()
        self.CLOSED = {}
        #CLOSED2 = []

        self.OPEN.put((self.env.fValue(startNode), startNode))

        # DUMMY Planning Stage.
        currNode = startNode
        while(not self.OPEN.empty()):
            priority, currNode = self.OPEN.get()
            #print( "Expanded fvalue: %f", self.env.fValue(currNode) )
            if currNode in self.CLOSED:
                continue
            else:
                nodeId = currNode.getNodeId()
                self.CLOSED[nodeId] = 1
            if self.env.gValue(goalNode) <= self.env.fValue(currNode):
                break

            self._expand2(currNode)

        print("Nodes expaneded", len(self.CLOSED))

        if currNode.getNodeId() != self.goalNode.getNodeId():
            # No plan found.
            return 0
        """
        else:
            # 1. Check if any island is in the discovered path.
            # 2. Complete the edge to the island.
            # Currently assumes only one island edge.

            currNode = goalNode
            while(currNode != startNode):
                if(currNode in self.env.activeIslandNodes):
                    break
                currNode = currNode.getParent()
            if currNode == startNode:
                # No island node in path.
                return 1

            # currNode is an island
            pseudoGoalNode = currNode
            pseudostartNode = currNode.getParent()

            ### LOG
            print("Plan found. Beginning correction of the plan.")
            print(currNode.getNodeId())

            openQ.put((pseudostartNode.getG() + pseudostartNode.getH(),
                pseudostartNode))

            while(not openQ.empty() and currNode != self.goalNode):
                priority, currNode = openQ.get()
                print(currNode.getNodeId())
                # Previously closed nodes might need to be re-expanded.
                if currNode in closed:
                    continue
                closed.append(currNode)

                #if viz is not None:
                viz.markPoint(self.env.getPointFromId(currNode.getNodeId()), 0)
                viz.displayImage(1)

                children, edgeCosts = self.env.getChildrenAndCosts(currNode)
                for child, edgeCost in zip(children, edgeCosts):
                    if child in closed2:
                        continue

                    if child.getH() == float("inf"):
                        child.setH(self.env.heuristic(child, goalNode))

                    #print(currNode.getH())
                    # Check for dummy value is inside update function.
                    updated = self.updateDummyG(child, currNode.getG() + edgeCost)
                    if updated:
                        child.setParent(currNode)
                        #XXX What if this node is already in the open list?
                        openQ.put((child.getG() + child.getH(), child))

        if currNode.getNodeId() == self.goalNode.getNodeId():
            print("Plan found")
            return 1
        else:
            print("Planning failed")
            return 0
        """
