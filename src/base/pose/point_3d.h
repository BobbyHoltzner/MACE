#ifndef POINT_3D_H
#define POINT_3D_H

#include "Eigen/Dense"
#include "point.h"

namespace mace {
namespace pose {

class Point3D : public Point<Point3D>
{

public:
    //!
    //! \brief Point3D
    //!
    Point3D()
    {
        this->x = 0.0;
        this->y = 0.0;
        this->z = 2.5;
    }

    //!
    //! \brief Point3D
    //! \param copy
    //!
    Point3D(const Point3D &copy)
    {
        this->x = copy.x;
        this->y = copy.y;
    }


    //!
    //! \brief Point2D
    //! \param posX
    //! \param posY
    //! \param posZ
    //!
    Point3D(const double &posX, const double &posY)
    {
        this->setX(posX);
        this->setY(posY);
    }
};

} //end of namespace pose
} //end of namespace base

#endif // POINT_3D_H
