#ifndef LATTICE_H
#define LATTICE_H

#include "types.h"


namespace hsearch {
/**
 * The Lattice class supports two public methods:
 *  - succs(*)
 *  - edgeCost(*)
 * that are required to traverse the graph.
 */
    class Lattice {
        public:
        Lattice();

        virtual Node Succs( const Node a, const Actions actions ) = 0;
        virtual float EdgeCost( const Node a, const Node b ) = 0;

        bool setActions( const Actions actions );

        Actions m_actions;
    };
} //namespace hsearch

#endif
