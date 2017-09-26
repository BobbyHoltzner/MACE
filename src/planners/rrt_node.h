#ifndef RRT_NODE_H
#define RRT_NODE_H

#include "base/state_space/state.h"

namespace mace {
namespace planners_sampling{

class RootNode : public state_space::State
{
public:
    RootNode():
        currentState(nullptr), parentNode(nullptr)
    {

    }

    ~RootNode() = default; //we do not have to destroy anything here as we did not new the states

public:

    void setCurrentState(state_space::State* state) { this->currentState = state; }
    void setParentNode(RootNode* node) { this->parentNode = node; }

    state_space::State* getCurrentState(){ return this->currentState; }
    RootNode* getParentNode(){ return this->parentNode; }

private:
    state_space::State* currentState;
    RootNode* parentNode;
};

} //end of namespace planners_sampling
} //end of namespace mace
#endif // RRT_NODE_H
