#ifndef PLANNER_H
#define PLANNER_H

#include "lattice.h"

namespace hsearch {
    /**
     * Base class for all planners.
     */
    class Planner {
        public:
        Planner();

        virtual bool plan( float allocated_time_sec, std::vector<Node> &path );
    };
}

#endif
