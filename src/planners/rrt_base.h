#ifndef RRT_BASE_H
#define RRT_BASE_H

namespace mace {
namespace planners_sampling{

class RRTBase{

    RRTBase()
    {

    }

    void setGoalProbability(const double &probability);

    void setMaxBranchLength(const double &length);

private:
    double m_goalProbability;
    double m_maxBranchLength;
};


} //end of namespace planners_sampling
} //end of namespace mace


#endif // RRT_BASE_H
