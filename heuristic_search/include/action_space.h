
using namespace std;
using Action = vector<double>;
using Actions = vector<Action>;

class ActionSpace {
    public:
    ActionSpace();

    bool addAction(vector<double> delta);
    vector<Action> getActions();

    private:
    vector<Action> actions;
};
