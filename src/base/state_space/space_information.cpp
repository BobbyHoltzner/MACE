#include "space_information.h"

namespace mace {
namespace state_space {

//consider std::move of the shared pointer here
SpaceInformation::SpaceInformation(StateSpacePtr space):
    m_stateSpace(space)
{

}

const StateSpacePtr& SpaceInformation::getStateSpace() const
{
    return m_stateSpace;
}

bool SpaceInformation::isStateValid(const State *state) const
{
    //for now lets just default this to true to see it work
    return true;
}

bool SpaceInformation::isEdgeValid(const State *lhs, const State *rhs) const
{
    //for now lets just default this to true to see it work
    return true;
}

double SpaceInformation::distanceBetween(const State *lhs, const State *rhs) const
{
    m_stateSpace->distanceBetween(lhs, rhs);
}

//!
//! \brief getNewState
//! \return
//!
State* SpaceInformation::getNewState() const
{
    return m_stateSpace->getNewState();
}

//!
//! \brief removeState
//! \param state
//!
void SpaceInformation::removeState(State* state) const
{
    m_stateSpace->removeState(state);
}

//!
//! \brief copyState
//! \param state
//! \return
//!
State* SpaceInformation::copyState(const State* state) const
{
    return m_stateSpace->copyState(state);
}

//!
//! \brief removeStates
//! \param states
//!
void SpaceInformation::removeStates(std::vector<State*> states) const
{
    m_stateSpace->removeStates(states);
}

} //end of namespace state_space
} //end of namespace mace
