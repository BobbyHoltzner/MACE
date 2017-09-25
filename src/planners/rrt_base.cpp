#include "rrt_base.h"

namespace mace {
namespace planners_sampling{

void RRTBase::setGoalProbability(const double &probability)
{
    m_goalProbability = probability;
}

void RRTBase::setMaxBranchLength(const double &length)
{
    m_maxBranchLength = length;
}

} //end of namespace planners_sampling
} //end of namespace mace
