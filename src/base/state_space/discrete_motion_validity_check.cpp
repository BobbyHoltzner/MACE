#include "discrete_motion_validity_check.h"

namespace mace {
namespace state_space {

DiscreteMotionValidityCheck::DiscreteMotionValidityCheck(const StateSpacePtr &space):
    AbstractMotionValidityCheck(space)
{

}

bool DiscreteMotionValidityCheck::isValid(const State *begin, const State *end) const
{

}

} //end of namespace state_space
} //end of namespace mace
