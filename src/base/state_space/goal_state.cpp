#include "goal_state.h"

namespace mace {
namespace state_space {

GoalState::GoalState(StateSpacePtr &space, const double &value):
    GoalSampler(space,value)
{
    this->setSampleFunction([this](State* sample)
    {
        stateSpace->getNewState();
        sample = goalState;
    });
}

void GoalState::setState(State* state)
{
    goalState = state;
}

const State* GoalState::getState() const
{
    return goalState;
}

void GoalState::sampleGoal(State* sample)
{
    sampleFunction(sample);
}

} //end of namespace state_space
} //end of namespace mace
