#include "cartesian_position_2D.h"

namespace mace{
namespace pose{

double CartesianPosition_2D::deltaX(const CartesianPosition_2D &that) const
{
    return this->getXPosition() - that.getXPosition();
}

double CartesianPosition_2D::deltaY(const CartesianPosition_2D &that) const
{
    return this->getYPosition() - that.getYPosition();
}

double CartesianPosition_2D::distanceBetween2D(const CartesianPosition_2D &pos) const
{
    double deltaX = this->data.getX() - pos.data.getX();
    double deltaY = this->data.getY() - pos.data.getY();
    double distance = sqrt(pow(deltaX,2) + pow(deltaY,2));
    return distance;
}

double CartesianPosition_2D::distanceTo(const CartesianPosition_2D &pos) const
{
    return this->distanceBetween2D(pos);
}

//!
//! \brief CartesianPosition_2D::polarBearingTo
//! \param pos
//! \return polar
//!
double CartesianPosition_2D::polarBearingTo(const CartesianPosition_2D &pos) const
{
    return atan2(deltaY(pos),deltaX(pos));
}

//!
//! \brief CartesianPosition_2D::compassBearingTo
//! \param pos
//! \return polar
//!
double CartesianPosition_2D::compassBearingTo(const CartesianPosition_2D &pos) const
{
    return math::correctBearing(atan2(deltaY(pos),deltaX(pos)));
}

//!
//! \brief CartesianPosition_2D::newPositionFromPolar
//! \param distance
//! \param bearing
//! \return
//!
CartesianPosition_2D CartesianPosition_2D::newPositionFromPolar(const double &distance, const double &bearing) const
{
    double newX = this->getXPosition() + cos(bearing) * distance;
    double newY = this->getYPosition() + sin(bearing) * distance;

    CartesianPosition_2D pos(newX, newY);
    return pos;
}

//!
//! \brief CartesianPosition_2D::newPositionFromCompass
//! \param distance
//! \param bearing
//! \return
//!
CartesianPosition_2D CartesianPosition_2D::newPositionFromCompass(const double &distance, const double &bearing) const
{
    double polarBearing = -bearing + 90;
    return newPositionFromPolar(distance,polarBearing);
}


std::ostream& operator<<(std::ostream& os, const CartesianPosition_2D& t)
{
    std::stringstream stream;
    stream.precision(6);
    stream << std::fixed << "Cartesian Position 2D: "<<
              t.getXPosition() << ", "<< t.getYPosition() << ".";
    os << stream.str();

    return os;
}


} //end of namespace pose
} //end of namespace mace
