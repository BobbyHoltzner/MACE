#ifndef POINT_H
#define POINT_H

#include "base_point.h"

namespace base{
namespace pose {

template <class DERIVED>
class Point : public BasePoint<DERIVED>
{

    bool operator<(const Point<DERIVED>& rhs)
    {
        if(this->x < rhs.x)
        {
            return true;
        }else
        {
            if(!this->is3D())
                return this->y < rhs.getY();
            else if(this->y < rhs.getY())
                return true;
            else
            {
                double thisZ = static_cast<const DERIVED*>(this)->getZ();
                double rhsZ  = static_cast<const DERIVED*>(rhs)->getZ();
                return thisZ < rhsZ;
            }
        }
    }
};

} //end of namespace pose
} //end of namespace base
#endif // POINT_H
