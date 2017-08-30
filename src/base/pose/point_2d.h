#ifndef POINT_2D_H
#define POINT_2D_H

#include "Eigen/Dense"
#include "point.h"

namespace mace {
namespace pose {

class Point2D : public Point<Point2D>
{

public:
    //!
    //! \brief Point2D
    //!
    Point2D()
    {
        this->x = 0.0;
        this->y = 0.0;
    }

    //!
    //! \brief Point2D
    //! \param copy
    //!
    Point2D(const Point2D &copy)
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
    Point2D(const double &posX, const double &posY)
    {
        this->setX(posX);
        this->setY(posY);
    }
};

} //end of namespace pose
} //end of namespace base

#endif // POINT_2D_H
