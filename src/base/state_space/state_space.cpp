#include "state_space.h"
namespace mace{
namespace state_space {

StateSpace::StateSpace()
{
    m_name = "";
}

StateSpace::~StateSpace()
{

}

bool StateSpace::interpolateStates(const State *begin, const State *end, const double &distance, State** interState)
{
    *interState = begin->getClone();
    return false;
}

} //end of namespace state
} //end of namespace mace
