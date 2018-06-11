#include "geodetic_position_2D.h"

namespace mace{
namespace pose{

double GeodeticPosition_2D::deltaLatitude(const GeodeticPosition_2D &that) const
{
    return this->getLatitude() - that.getLatitude();
}

double GeodeticPosition_2D::deltaLongitude(const GeodeticPosition_2D &that) const
{
    return this->getLongitude() - that.getLongitude();
}

double GeodeticPosition_2D::distanceBetween2D(const GeodeticPosition_2D &pos) const
{

}

double GeodeticPosition_2D::distanceTo(const GeodeticPosition_2D &pos) const
{
    return this->distanceBetween2D(pos);
}

//!
//! \brief GeodeticPosition_2D::polarBearingTo
//! \param pos
//! \return polar
//!
double GeodeticPosition_2D::polarBearingTo(const GeodeticPosition_2D &pos) const
{

}

//!
//! \brief GeodeticPosition_2D::compassBearingTo
//! \param pos
//! \return polar
//!
double GeodeticPosition_2D::compassBearingTo(const GeodeticPosition_2D &pos) const
{

}

//!
//! \brief GeodeticPosition_2D::newPositionFromPolar
//! \param distance
//! \param bearing
//! \return
//!
GeodeticPosition_2D GeodeticPosition_2D::newPositionFromPolar(const double &distance, const double &bearing) const
{

}

//!
//! \brief GeodeticPosition_2D::newPositionFromCompass
//! \param distance
//! \param bearing
//! \return
//!
GeodeticPosition_2D GeodeticPosition_2D::newPositionFromCompass(const double &distance, const double &bearing) const
{

}

void GeodeticPosition_2D::applyPositionalShiftFromPolar(const double &distance, const double &bearing)
{

}

void GeodeticPosition_2D::applyPositionalShiftFromCompass(const double &distance, const double &bearing)
{

}

} //end of namespace pose
} //end of namespace mace
