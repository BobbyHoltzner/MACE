#include "global_position.h"

GlobalPosition::GlobalPosition()
{
    this->latitude = 0.0;
    this->longitude = 0.0;
    this->altitude = 0.0;
}

GlobalPosition::GlobalPosition(const double &latitude, const double &longitude, const double &altitude)
{
    this->latitude = latitude;
    this->longitude = longitude;
    this->altitude = altitude;
}

void GlobalPosition::setPosition(const double &latitude, const double &longitude, const double &altitude)
{
    this->latitude = latitude;
    this->longitude = longitude;
    this->altitude = altitude;
}

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

double GlobalPosition::initialBearing(const GlobalPosition &position){
    return 0.0;
    //return (bearingBetween(position) + 360.0) % 360.0;
}

double GlobalPosition::finalBearing(const GlobalPosition &position){
    return 0.0;
    //return (bearingBetween(position) + 180.0) % 360.0;
}

double GlobalPosition::distanceBetween(const GlobalPosition &position)
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

double GlobalPosition::getAltitude() const
{
    return altitude;
}

double GlobalPosition::getLatitude() const
{
    return latitude;
}

double GlobalPosition::getLongitude() const
{
    return longitude;
}

double GlobalPosition::convertDegreesToRadians(const double &degrees)
{
    double pi = 3.14159265358979323846;
    double radians = degrees * (pi/180.0);
    return radians;
}

double GlobalPosition::convertRadiansToDegrees(const double &radians)
{
    double pi = 3.14159265358979323846;
    double degrees = radians * (180.0/pi);
    return degrees;
}
