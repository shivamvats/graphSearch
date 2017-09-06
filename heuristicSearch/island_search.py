from astar import *
import time

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
        self.i = 0

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

    def updateGvalue2(self, node, newG, newH1):
        #print( node.getGvalue(), newG, newH1 )
        if(node.getG() + 5*node.getH1() > newG + newH1):
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
        self.i += 1
        self.viz.markPoint(self.env.getPointFromId(node.getNodeId()), 0)
        if self.i%1000 == 0:
            self.viz.displayImage(1)

        succs, dummySuccs, edgeCosts, dummyCosts = \
            self.env.getChildrenIslandsAndCosts(node)

        if dummySuccs:
            # If there is an dummy edge, do not expand any other neighbours.
            succs, edgeCosts = [], []

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

    def _expand3(self, node, island=None):
        """Modified expand method that also allows for dummy edges to nearby
        islands."""
        #if node in self.env.activeIslandNodes:
        #    print("Island node expanded", "g = ", node.getG())
        REFINEMENT_DEBUG = 0

        #if viz is not None:
        self.viz.markPoint(self.env.getPointFromId(node.getNodeId()), 0)
        self.viz.displayImage(1)

        succs, edgeCosts  = \
            self.env.getChildrenAndCosts(node)
        # Delete nodes falling outside the island's activation region.
        if island:
            toBeDeleted = []
            for i in range( len( succs ) ):
                if ( self.env.distanceBetweenNodes( succs[i].getNodeId(),
                        island.getNodeId() ) > self.env.islandThresh ):
                    toBeDeleted.append( i )
            for i in toBeDeleted[::-1]:
                del succs[i]
                del edgeCosts[i]

        def _insertInOpen( node, succ, island ):
            succ.setParent( node )
            self.OPEN.put( (self.env.fValue( succ, island), succ) )
        def _insideRegion( node, island ):
            print( self.env.distanceBetweenNodes( node.getNodeId(),
                island.getNodeId() ) )
            return self.env.distanceBetweenNodes( node.getNodeId(), island.getNodeId() ) <= self.env.islandThresh

        if REFINEMENT_DEBUG:
            print( "parent", self.env.getPointFromId( node.getNodeId()) )
            print( _insideRegion( node, island ) )
        for succ, cost in zip(succs, edgeCosts):

            if REFINEMENT_DEBUG:
                print( self.env.getPointFromId(succ.getNodeId()) )
                print( _insideRegion( succ, island) )
                print( "gvalue updated", succ.getGvalue() )

            self.viz.drawCircle(self.env.getPointFromId(succ.getNodeId()), 2)
            self.viz.displayImage(1)
            updated = self.updateGvalue2(succ, node.getG() + cost, node.getH1())
            if updated:
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
            if self.i % 500 == 0:
                print( self.OPEN.qsize() )
            priority, currNode = self.OPEN.get()
            #print( "Expanded fvalue: %f", self.env.fValue(currNode) )
            if self.env.gValue(goalNode) <= self.env.fValue(currNode):
                break
            if currNode in self.CLOSED:
                continue

            #flag = 0
            #for island in self.env.activeIslandNodes:
            #    if  ( (self.env.distanceBetweenNodes( currNode.getNodeId(),
            #        island.getNodeId() ) < self.env.islandThresh) and
            #        currNode.getH1() < island.getH1() ):
            #        flag = 1
            #        break
            #if flag:
            #    continue

            nodeId = currNode.getNodeId()
            self.CLOSED[nodeId] = 1

            self._expand2(currNode)

        print("Nodes expanded", len(self.CLOSED))

        if currNode.getNodeId() != self.goalNode.getNodeId():
            # No plan found.
            return 0

        ### Refinement Stage.
        ### ----------------
        # 1. Find the optimal path P and the first island on it.
        # 2. Complete the edge to the island.
        # Currently assumes only one island edge.

        path = self.env.retrievePath( startNode, goalNode )
        print("Initial plan found. Beginning refinement of the plan.")

        def _findDummyEdge( path ):
            """Finds the first dummy edge."""
            dummyStart, dummyGoal = None, None
            for node in path:
                if node.getH1() > 0:
                    # First dummy edge.
                    dummyGoal = node
                    dummyStart = node.getParent()
                    break
            if (dummyStart is None) ^ (dummyGoal is None):
                raise ValueError( "The dummy edge nodes are fishy.")
            return (dummyStart, dummyGoal)

        self.CLOSED.clear()
        self.OPEN = Q.PriorityQueue()
        dummyStart, dummyGoal = _findDummyEdge( path )
        while( not( dummyGoal is None ) ):
            print( map( lambda n: n.getNodeId(), self.env.activeIslandNodes ) )
            print( "trying to remove", dummyGoal.getNodeId() )
            self.env.activeIslandNodes.remove( dummyGoal )
            print( dummyGoal.getNodeId(), "is removed")
            print( "\n" )
            DUMMYCOST = dummyGoal.getH1()
            #print( map( lambda n: n.getNodeId(), self.env.activeIslandNodes ) )

            self.OPEN.put( (self.env.fValue( dummyStart, dummyGoal ), dummyStart) )

            while( not self.OPEN.empty() ):
                priority, currNode = self.OPEN.get()
                nodeId = currNode.getNodeId()
                #print(nodeId, priority)
                if nodeId == dummyGoal.getNodeId():
                    break
                #print( "Expanded fvalue: %f", self.env.fValue(currNode) )
                # the dummy edge's refinement may be too expensive.
                #if self.env.gValue(goalNode) <= self.env.fValue(currNode):
                    #break
                if currNode in self.CLOSED:
                    continue

                self.CLOSED[nodeId] = 1

                self._expand3(currNode, dummyGoal)
            print( "One dummy edge refined." )
            for node in path:
                if node.getH1() > 0:
                    node.setH1( node.getH1() - DUMMYCOST )
            # XXX Testing purpose.
            #dummyGoal = None
            dummyStart, dummyGoal = _findDummyEdge( path )

        print("Plan found")
        return 1
