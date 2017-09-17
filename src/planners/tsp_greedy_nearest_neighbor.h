#ifndef TSP_GREEDY_NEAREST_NEIGHBOR_H
#define TSP_GREEDY_NEAREST_NEIGHBOR_H

#include <iostream>
#include <vector>
#include <limits>

#include "base/pose/abstract_position.h"
#include "base/pose/cartesian_position_2D.h"

namespace mace {
namespace planners{

template <class T>
class TSP_GreedyNearestNeighbor
{
    //need to make sure it is of a base type that we can understand so that we may call the overloaded virtual
    //distance function
    //static_assert(std::is_base_of<pose::AbstractPosition, T>::value,"T must be a descendant of AbstractPosition");
public:
    TSP_GreedyNearestNeighbor() = default;

    void updateSites(const std::vector<T> &sites);

    void clearSites();

    std::vector<T> executeTSP(const T &start);


private:
    std::vector<T*> copy_sites();

private:
    std::vector<T> m_siteNodes;
};

} //end of namespace planners
} //end of namespace mace

#endif // TSP_GREEDY_NEAREST_NEIGHBOR_H
