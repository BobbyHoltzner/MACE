#include "point_3d.h"

namespace base {
namespace pose {

//!
//! \brief Point3D::Point3D
//!
Point3D::Point3D():
    Point2D()
{

}

//!
//! \brief Point3D::~Point3D
//!
Point3D::~Point3D()
{

}

//!
//! \brief Point3D::Point3D
//! \param copy
//!
Point3D::Point3D(const Point3D &copy):
    Point2D(copy)
{
    this->z = copy.z;
    this->posZFlag = copy.posZFlag;
}

Point3D::Point3D(const Point2D &copy):
    Point2D(copy)
{

}

Point3D::Point3D(const Point2D &copy, const double &posZ):
    Point2D(copy)
{
    this->setZ(posZ);
}

//!
//! \brief Point3D::Point3D
//! \param posX
//! \param posY
//! \param posZ
//!
Point3D::Point3D(const double &posX, const double &posY, const double &posZ):
    Point2D(posX, posY)
{
    this->setZ(posZ);
}

} //end of namepsace pose
} //end of namespace base
