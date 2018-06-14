#ifndef RRT_NODE_H
#define RRT_NODE_H

#include "base/state_space/state_space.h"
namespace mace {
namespace planners_sampling{

class RootNode
{
public:
    RootNode(const state_space::StateSpacePtr &stateSpace):
        currentState(stateSpace->getNewState()), parentNode(nullptr)
    {

    }

    RootNode(state_space::State* state):
        currentState(state), parentNode(nullptr)
    {

    }

    ~RootNode() = default; //we do not have to destroy anything here as we did not new the states

public:
    void setCurrentState(state_space::State* state) { this->currentState = state; }
    void setParentNode(RootNode* node) { this->parentNode = node; }

    state_space::State* getCurrentState()const { return this->currentState; }
    RootNode* getParentNode()const { return this->parentNode; }

private:
    state_space::State* currentState;
    RootNode* parentNode;
};

} //end of namespace planners_sampling
} //end of namespace mace
#endif // RRT_NODE_H
