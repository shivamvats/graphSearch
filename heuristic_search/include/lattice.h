

using Node = int;

/**
 * The Lattice class supports two public methods:
 *  - succs(*)
 *  - edgeCost(*)
 * that are required to traverse the graph.
 */
class Lattice {
    public:
    Lattice();

    virtual Node succs(const Node a, const Actions actions);
    virtual float edgeCost(const Node a, const Node b);

    bool setActions( const Actions actions );

    Actions m_actions;
};
