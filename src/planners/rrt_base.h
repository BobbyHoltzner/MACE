#ifndef RRT_BASE_H
#define RRT_BASE_H

#include <memory>

#include "planners.h"
#include "rrt_node.h"
#include "nearest_neighbor_abstract.h"

namespace mace {
namespace planners_sampling{

class RRTBase : public planners::Planners{

    RRTBase()
    {

    }

    void solve() override;

public:
//    template<class NN>
//    void setNearestNeighbor()
//    {
//        m_nnStrategy = std::make_shared<NN<RootNode*>>();
//    }

    /** The basis for the following functions is that these are base paramters
        required for any tree based search. */
public:
    void setGoalProbability(const double &probability);

    double getGoalProbability() const;

    void setMaxBranchLength(const double &length);

    double getMaxBranchLength() const;

protected:

    std::shared_ptr<nn::NearestNeighborAbstract<RootNode*>> m_nnStrategy;
    /**
     * @brief The probability of biasing samples towards the goal to ensure the tree
     * indeed expands and tries to connect to the goal rather than randomly through
     * the space.
     */
    double m_goalProbability;

    /**
     * @brief The maximum branch length used to connect from a root of the tree to
     * the random sampled point. This length will determine where a new root of the
     * tree is added between the two points.
     */
    double m_maxBranchLength;
};


} //end of namespace planners_sampling
} //end of namespace mace


#endif // RRT_BASE_H
