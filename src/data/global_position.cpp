#include "global_position.h"

namespace Data {

/**
 * @brief GlobalPosition::GlobalPosition
 */
GlobalPosition::GlobalPosition()
{
    this->latitude = 0.0;
    this->longitude = 0.0;
    this->altitude = 0.0;
}

/**
 * @brief GlobalPosition::GlobalPosition
 * @param latitude
 * @param longitude
 * @param altitude
 */
GlobalPosition::GlobalPosition(const double &latitude, const double &longitude, const double &altitude)
{
    this->latitude = latitude;
    this->longitude = longitude;
    this->altitude = altitude;
}

/**
 * @brief GlobalPosition::GlobalPosition
 * @param copyObj
 */
GlobalPosition::GlobalPosition(const GlobalPosition &copyObj)
{
    this->latitude = copyObj.getLatitude();
    this->longitude = copyObj.getLongitude();
    this->altitude = copyObj.getAltitude();
}

/**
 * @brief GlobalPosition::setPosition
 * @param latitude
 * @param longitude
 * @param altitude
 */
void GlobalPosition::setPosition(const double &latitude, const double &longitude, const double &altitude)
{
    this->latitude = latitude;
    this->longitude = longitude;
    this->altitude = altitude;
}


/**
 * @brief GlobalPosition::NewPositionFromHeadingBearing
 * @param distance
 * @param bearing
 * @param degreesFlag
 * @param newPosition
 */
void GlobalPosition::NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag, GlobalPosition &newPosition)
{
    double earthRadius = 6371000; //approximate value in meters
    double latitudeRad = convertDegreesToRadians(latitude);
    double longitudeRad = convertDegreesToRadians(longitude);
    double bearingRad = 0.0;

    if(degreesFlag == true){
        bearingRad = convertDegreesToRadians(bearing);
    }else{
        bearingRad = bearing;
    }
    double distanceRatio = distance / earthRadius;

    double newLat = asin(sin(latitudeRad) * cos(distanceRatio) + cos(latitudeRad) * sin(distanceRatio) * cos(bearingRad));
    double newLon = longitudeRad + atan2(sin(bearingRad) * sin(distanceRatio) * cos(latitudeRad),
                                         cos(distanceRatio) - sin(latitudeRad) * sin(newLat));

    newPosition.setPosition(convertRadiansToDegrees(newLat),convertRadiansToDegrees(newLon),newPosition.getAltitude());
}

/**
 * @brief GlobalPosition::bearingBetween
 * @param position
 * @return
 */
double GlobalPosition::bearingBetween(const GlobalPosition &position)
{
    double deltaLatitude = convertDegreesToRadians(position.getLatitude() - this->latitude);
    double deltaLongitude = convertDegreesToRadians(position.getLongitude() - this->longitude);

    double tmpY = sin(deltaLongitude) * cos(convertDegreesToRadians(position.getLatitude()));
    double tmpX = cos(convertDegreesToRadians(this->latitude)) * sin(convertDegreesToRadians(position.getLatitude())) -
            sin(convertDegreesToRadians(this->latitude)) * sin(convertDegreesToRadians(position.getLatitude())) *
            cos(deltaLongitude);
    double bearing = atan2(tmpY,tmpX);

    return convertRadiansToDegrees(bearing);
}

/**
 * @brief GlobalPosition::initialBearing
 * @param position
 * @return
 */
double GlobalPosition::initialBearing(const GlobalPosition &position){
    return 0.0;
    //return (bearingBetween(position) + 360.0) % 360.0;
}

/**
 * @brief GlobalPosition::finalBearing
 * @param position
 * @return
 */
double GlobalPosition::finalBearing(const GlobalPosition &position){
    return 0.0;
    //return (bearingBetween(position) + 180.0) % 360.0;
}

/**
 * @brief GlobalPosition::distanceBetween2D
 * @param position
 * @return
 */
double GlobalPosition::distanceBetween2D(const GlobalPosition &position)
{
    double earthRadius = 6371000; //approximate value in meters
    double deltaLatitude = convertDegreesToRadians(position.getLatitude() - this->latitude);
    double deltaLongitude = convertDegreesToRadians(position.getLongitude() - this->longitude);

    double tmpA = sin(deltaLatitude/2) * sin(deltaLatitude/2) +
            cos(this->latitude) * cos(position.getLatitude()) *
            sin(deltaLongitude/2) * sin(deltaLongitude/2);

    double tmpC = 2 * atan2(sqrt(tmpA),sqrt(1-tmpA));

    double distance = earthRadius * tmpC;

    return distance;
}

/**
 * @brief GlobalPosition::distanceBetween3D
 * @param position
 * @return
 */
double GlobalPosition::distanceBetween3D(const GlobalPosition &position)
{
    double distance2D = this->distanceBetween2D(position);
    double deltaAltitude = abs(this->altitude - position.getAltitude());
    return(sqrt(deltaAltitude * deltaAltitude + distance2D * distance2D));
}


/**
 * @brief GlobalPosition::getAltitude
 * @return
 */
double GlobalPosition::getAltitude() const
{
    return altitude;
}

double GlobalPosition::setAltitude(const double &altitude)
{
    this->altitude = altitude;
}

/**
 * @brief GlobalPosition::getLatitude
 * @return
 */
double GlobalPosition::getLatitude() const
{
    return latitude;
}
double GlobalPosition::setLatitude(const double &latitude)
{
    this->latitude = latitude;
}

/**
 * @brief GlobalPosition::getLongitude
 * @return
 */
double GlobalPosition::getLongitude() const
{
    return longitude;
}

double GlobalPosition::setLongitude(const double &longitude)
{
    this->longitude = longitude;
}

/**
 * @brief GlobalPosition::convertDegreesToRadians
 * @param degrees
 * @return
 */
double GlobalPosition::convertDegreesToRadians(const double &degrees)
{
    double pi = 3.14159265358979323846;
    double radians = degrees * (pi/180.0);
    return radians;
}

/**
 * @brief GlobalPosition::convertRadiansToDegrees
 * @param radians
 * @return
 */
double GlobalPosition::convertRadiansToDegrees(const double &radians)
{
    double pi = 3.14159265358979323846;
    double degrees = radians * (180.0/pi);
    return degrees;
}

} //end of namespace Data
