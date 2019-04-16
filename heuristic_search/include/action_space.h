#include "types.h"

namespace hsearch {
    class ActionSpace {
        public:
        ActionSpace();

        bool addAction(std::vector<double> delta);
        vector<Action> getActions();

        std::vector<Action> m_actions;
    };
} // namespace hsearch
