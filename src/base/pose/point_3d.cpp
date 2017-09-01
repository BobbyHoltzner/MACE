#include "point_3d.h"

namespace mace{
namespace pose{

//!
//! \brief Point3D
//!
Point3D::Point3D()
{

}

Point3D::Point3D(const Point2D &copy):
    Point2D(copy)
{

}

Point3D::Point3D(const Point2D &copy, const double &z):
    Point2D(copy)
{
    this->setZ(z);
}

//!
//! \brief Point3D
//! \param copy
//!
Point3D::Point3D(const Point3D &copy):
    Point2D(copy)
{
    this->z = copy.z;
    this->posZFlag = copy.posZFlag;
}


//!
//! \brief Point2D
//! \param posX
//! \param posY
//! \param posZ
//!
Point3D::Point3D(const double &posX, const double &posY, const double &posZ):
    Point2D(posX, posY)
{
    this->setZ(posZ);
}


Point2D Point3D::get2D() const
{
    return Point2D(*this);
}

/** The following are defined in point_2d.h as friend functions*/

//!
//! \brief operator +
//! \param lhs
//! \param rhs
//! \return
//!
Point3D operator + (const Point2D &lhs, const Point3D &rhs)
{
    return rhs + lhs;
}

//!
//! \brief operator +
//! \param lhs
//! \param rhs
//! \return
//!
Point3D operator - (const Point2D &lhs, const Point3D &rhs)
{
    return rhs - lhs;
}

//!
//! \brief operator ==
//! \param lhs
//! \param rhs
//! \return
//!
bool operator == (const Point2D &lhs, const Point3D &rhs)
{
    return rhs == lhs;
}

//!
//! \brief operator <
//! \param lhs
//! \param rhs
//! \return
//!
bool operator < (const Point2D &lhs, const Point3D &rhs)
{
    return lhs < rhs.get2D();
}

//!
//! \brief operator >
//! \param lhs
//! \param rhs
//! \return
//!
bool operator > (const Point2D &lhs, const Point3D &rhs)
{
    return lhs > rhs.get2D();
}

} //end of namespace pose
} //end of namespace mace
