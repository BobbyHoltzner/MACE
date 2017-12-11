#ifndef CIRCLE_ITERATOR_H
#define CIRCLE_ITERATOR_H

#include "base/pose/cartesian_position_2D.h"

namespace mace{
namespace maps{

class CircleIterator
{
public:
    CircleIterator(const pose::CartesianPosition_2D &origin, const double &radius);

    CircleIterator& operator =(const CircleIterator &rhs);

    bool operator !=(const CircleIterator &rhs) const;

    CircleIterator& operator ++();

    bool isPastEnd() const;

private:
    bool isInside() const;

    void findSubmapParameters(const pose::CartesianPosition_2D &origin, const double &radius, int &startIndex);

};

} //end of namepsace mace
} //end of namespace maps

#endif // CIRCLE_ITERATOR_H
