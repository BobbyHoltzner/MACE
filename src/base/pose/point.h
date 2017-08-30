#ifndef POINT_H
#define POINT_H

#include "abstract_point.h"

namespace mace{
namespace pose {

template <class DERIVEDCLASS>
class Point : public AbstractPoint<DERIVEDCLASS>
{
    bool operator<(const Point<DERIVEDCLASS>& rhs)
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
                double thisZ = static_cast<const DERIVEDCLASS*>(this)->getZ();
                double rhsZ  = static_cast<const DERIVEDCLASS*>(rhs)->getZ();
                return thisZ < rhsZ;
            }
        }
    }
};

} //end of namespace pose
} //end of namespace base
#endif // POINT_H
