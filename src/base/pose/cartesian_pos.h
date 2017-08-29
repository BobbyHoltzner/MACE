#ifndef CARTESIAN_POS_H
#define CARTESIAN_POS_H

#include "abstract_position.h"

namespace base{
namespace pose{

class CartesianPos : public AbstractPosition<CartesianPos, Point2D>
{
public:
    CartesianPos();
};

} //end of namespace pose
} //end of namespace base

#endif // CARTESIAN_POS_H
