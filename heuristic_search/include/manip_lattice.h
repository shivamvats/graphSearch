#ifndef MANIP_LATTICE_H
#define MANIP_LATTICE_H

#include "lattice.h"

namespace hsearch {
    /**
    * Defines a lattice to be used for planning for high dimensional manipulation.
    */
    class ManipLattice : public Lattice {
        public:
        ManipLattice();

        Node Succs( const Node a, const Actions actions ) override;
        float EdgeCost( const Node a, const Node b ) override;
    };
} //namespace hsearch

#endif
