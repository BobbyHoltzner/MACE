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
    double originLatitude = convertDegreesToRadians(this->getLatitude());
    double originLongitude = convertDegreesToRadians(this->getLongitude());
    double finalLatitude = convertDegreesToRadians(pos.getLatitude());
    double finalLongitude = convertDegreesToRadians(pos.getLongitude());

    double deltaLongitude = finalLongitude - originLongitude;

    double tmpY = sin(deltaLongitude) * cos(finalLatitude);
    double tmpX = cos(originLatitude) * sin(finalLatitude) -
            sin(originLatitude) * cos(finalLatitude) *
            cos(deltaLongitude);
    double bearing = atan2(tmpY,tmpX);
    return bearing;
}

//!
//! \brief GeodeticPosition_2D::compassBearingTo
//! \param pos
//! \return polar
//!
double GeodeticPosition_2D::compassBearingTo(const GeodeticPosition_2D &pos) const
{
    return correctBearing(polarBearingTo(pos));
}

//!
//! \brief GeodeticPosition_2D::newPositionFromPolar
//! \param distance
//! \param bearing
//! \return
//!
GeodeticPosition_2D GeodeticPosition_2D::newPositionFromPolar(const double &distance, const double &bearing) const
{
    double compassDirection = -90 + bearing;
    return newPositionFromCompass(distance,wrapTo2Pi(compassDirection));
}

//!
//! \brief GeodeticPosition_2D::newPositionFromCompass
//! \param distance
//! \param bearing
//! \return
//!
GeodeticPosition_2D GeodeticPosition_2D::newPositionFromCompass(const double &distance, const double &bearing) const
{
    double earthRadius = 6371000; //approximate value in meters
    double latitudeRad = convertDegreesToRadians(this->getLatitude());
    double longitudeRad = convertDegreesToRadians(this->getLongitude());

    double distanceRatio = distance / earthRadius;

    double newLat = asin(sin(latitudeRad) * cos(distanceRatio) + cos(latitudeRad) * sin(distanceRatio) * cos(bearing));
    double newLon = longitudeRad + atan2(sin(bearing) * sin(distanceRatio) * cos(latitudeRad),
                                         cos(distanceRatio) - sin(latitudeRad) * sin(newLat));

    GeodeticPosition_2D newPos(convertRadiansToDegrees(newLat),convertRadiansToDegrees(newLon));

    return newPos;
}

void GeodeticPosition_2D::applyPositionalShiftFromPolar(const double &distance, const double &bearing)
{

}

void GeodeticPosition_2D::applyPositionalShiftFromCompass(const double &distance, const double &bearing)
{

}

} //end of namespace pose
} //end of namespace mace
