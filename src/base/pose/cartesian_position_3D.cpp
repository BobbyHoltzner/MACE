#include "cartesian_position_3D.h"

namespace mace{
namespace pose{

double CartesianPosition_3D::deltaX(const CartesianPosition_3D &that) const
{
    return this->getXPosition() - that.getXPosition();
}

double CartesianPosition_3D::deltaY(const CartesianPosition_3D &that) const
{
    return this->getYPosition() - that.getYPosition();
}

double CartesianPosition_3D::deltaZ(const CartesianPosition_3D &that) const
{
    return this->getZPosition() - that.getZPosition();
}

double CartesianPosition_3D::distanceBetween2D(const CartesianPosition_3D &pos) const
{
    double deltaX = this->data.getX() - pos.data.getX();
    double deltaY = this->data.getY() - pos.data.getY();
    double distance = sqrt(pow(deltaX,2) + pow(deltaY,2));
    return distance;
}

double CartesianPosition_3D::distanceTo(const CartesianPosition_3D &pos) const
{
    return this->distanceBetween2D(pos);
}

//!
//! \brief CartesianPosition_3D::polarBearingTo
//! \param pos
//! \return polar
//!
double CartesianPosition_3D::polarBearingTo(const CartesianPosition_3D &pos) const
{
    return atan2(deltaY(pos),deltaX(pos));
}

//!
//! \brief CartesianPosition_3D::polarBearingTo
//! \param pos
//! \return polar
//!
double CartesianPosition_3D::compassBearingTo(const CartesianPosition_3D &pos) const
{
    return math::correctBearing(atan2(deltaY(pos),deltaX(pos)));
}

//!
//! \brief CartesianPosition_3D::newPositionFromPolar
//! \param distance
//! \param bearing
//! \return
//!
CartesianPosition_3D CartesianPosition_3D::newPositionFromPolar(const double &distance, const double &bearing) const
{
    double newX = this->getXPosition() + cos(bearing) * distance;
    double newY = this->getYPosition() + sin(bearing) * distance;

    CartesianPosition_3D pos(newX, newY,this->getZPosition());
    return pos;
}

//!
//! \brief CartesianPosition_3D::newPositionFromCompass
//! \param distance
//! \param bearing
//! \return
//!
CartesianPosition_3D CartesianPosition_3D::newPositionFromCompass(const double &distance, const double &bearing) const
{
    double polarBearing = -bearing + 90;
    return newPositionFromPolar(distance,polarBearing);
}

} //end of namespace pose
} //end of namespace mace
