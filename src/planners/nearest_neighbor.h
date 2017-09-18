#ifndef NEAREST_NEIGHBOR_H
#define NEAREST_NEIGHBOR_H

#include <vector>
#include <functional>

namespace mace {
namespace planners{

template <class T>

class NearestNeighbors {

public:

    typedef std::function<double(const T &, const T &)> DistanceFunction;

    NearestNeighbors() = default;

    virtual ~NearestNeighbors() = default;

    virtual void setDistanceFunction(const DistanceFunction &distFun) {
        distFun_ = distFun;
    }

    const DistanceFunction &getDistanceFunction() const {
        return distFun_;
    }

    virtual bool reportsSortedResults() const = 0;

    virtual void clear() = 0;

    virtual void add(const T &data) = 0;

    virtual void add(const std::vector<T> &data)

    {

        for (auto elt = data.begin(); elt != data.end(); ++elt)

            add(*elt);

    }

    virtual bool remove(const T &data) = 0;

    virtual T nearest(const T &data) const = 0;

    virtual void nearestK(const T &data, std::size_t k, std::vector<T> &nbh) const = 0;

    virtual void nearestR(const T &data, double radius, std::vector<T> &nbh) const = 0;

    virtual std::size_t size() const = 0;

    virtual void list(std::vector<T> &data) const = 0;



protected:

    /** \brief The used distance function */

    DistanceFunction distFun_;

};

} //end of namespace planners
} //end of namespace mace

#endif // NEAREST_NEIGHBOR_H
