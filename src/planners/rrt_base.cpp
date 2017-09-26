#include "rrt_base.h"

namespace mace {
namespace planners_sampling{

void RRTBase::solve()
{


}

void RRTBase::setGoalProbability(const double &probability)
{
    m_goalProbability = probability;
}

double RRTBase::getGoalProbability() const{
    return this->m_goalProbability;
}

void RRTBase::setMaxBranchLength(const double &length)
{
    m_maxBranchLength = length;
}

double RRTBase::getMaxBranchLength() const
{
    return this->m_maxBranchLength;
}

} //end of namespace planners_sampling
} //end of namespace mace
