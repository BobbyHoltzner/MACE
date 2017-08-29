#include "point_2d.h"

namespace base {
namespace pose {

//!
//! \brief BasePoint
//!
Point2D::Point2D()
{

}

//!
//! \brief Point2D::~Point2D
//!
Point2D::~Point2D()
{

}

//!
//! \brief BasePoint
//! \param copy
//!
Point2D::Point2D(const Point2D &copy)
{
    this->x = copy.x;
    this->y = copy.y;
    this->posXFlag = copy.posXFlag;
    this->posYFlag = copy.posYFlag;
}

//!
//! \brief BasePoint
//! \param posX
//! \param posY
//!
Point2D::Point2D(const double &posX, const double &posY)
{
    this->x = posX;
    this->y = posY;
}


} //end of namepsace pose
} //end of namespace base
