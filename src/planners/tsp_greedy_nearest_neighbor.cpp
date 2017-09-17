#include "tsp_greedy_nearest_neighbor.h"

namespace mace {
namespace planners {

template <class T>
void TSP_GreedyNearestNeighbor<T>::updateSites(const std::vector<T> &sites)
{
    m_siteNodes = sites;
}

template <class T>
void TSP_GreedyNearestNeighbor<T>::clearSites()
{
    m_siteNodes.clear();
    m_siteNodes.shrink_to_fit();
}

template <class T>
std::vector<T> TSP_GreedyNearestNeighbor<T>::executeTSP(const T &start)
{
    size_t siteSize = m_siteNodes.size();

    std::vector<T> solution;
    solution.reserve(siteSize + 1);
    solution.push_back(start);

    std::vector<T*> siteList = copy_sites();

    while(siteList.size() != 0)
    {
        int baseIndex = solution.size() - 1;
        double currentMinCost = std::numeric_limits<double>::max();
        int currentMinIndex = 0;
        size_t searchSize = siteList.size();

        for(size_t i = 0; i < searchSize; i++)
        {
            //compute the cost to go from the current point to all the remaining points in the list
            //in this case the cost is just the euclidian distance as defined by the distance
            //function from the abstract position class.
            double distanceCost = solution[baseIndex].distanceTo(*siteList[i]);
            if(distanceCost < currentMinCost)
            {
                currentMinCost = distanceCost;
                currentMinIndex = i;
            }
        }
        solution.push_back(*siteList[currentMinIndex]);
        siteList.erase(siteList.begin() + currentMinIndex);
    }

    return solution;
}


template <class T>
std::vector<T*> TSP_GreedyNearestNeighbor<T>::copy_sites()
{

    int size = m_siteNodes.size();
    std::vector<T*> copy;
    copy.reserve(size);

    for (int i = 0; i < size; i++)
    {
        copy.push_back(&m_siteNodes[i]);
    }

    return copy;
}


template class TSP_GreedyNearestNeighbor<pose::Position<pose::CartesianPosition_2D>>;


} //end of namespace planners
} //end of namespace mace
