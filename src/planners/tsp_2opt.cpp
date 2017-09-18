#include "tsp_2opt.h"

namespace mace{
namespace planners {

template <class T>
TSP_2OPT<T>::TSP_2OPT():
    TSP_GreedyNearestNeighbor<T>()
{

}

template <class T>
double TSP_2OPT<T>::executeTSP(const T &start, std::vector<T> &tour)
{
    //double greedyLength = TSP_GreedyNearestNeighbor<T>::executeTSP(start, tour);

    std::vector<T*> backup = TSP_GreedyNearestNeighbor<T>::copy_sites(tour);
    double greedyLength = TSP_GreedyNearestNeighbor<T>::computeTourLength(backup);

    double currentLength = greedyLength;
    double newTourLength = 0.0;

    std::vector<T*> copyHold = TSP_GreedyNearestNeighbor<T>::copy_sites(tour);
    unsigned int tourLength = copyHold.size();
    unsigned int halfwayLength = (unsigned int)ceil(tourLength/2.0);
    //we can optimize this routine but for now lets just leave it
    for (unsigned int i = 1; i <= halfwayLength ; i++)
    {
        for (unsigned int j = i+1; j < tourLength; j++)
        {
            std::vector<T*> swap = copyHold;
            T* base = swap[i];
            T* iter = swap[j];

            //perform the swap
            swap[i] = iter;
            swap[j] = base;
            newTourLength = TSP_GreedyNearestNeighbor<T>::computeTourLength(swap);
            if(newTourLength < currentLength)
            {
                copyHold = swap;
                currentLength = newTourLength;
            }
        }
    }
}

template class TSP_2OPT<pose::Position<pose::CartesianPosition_2D>>;

} //end of namespace planners
} //end of namespace mace
