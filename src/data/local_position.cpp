#include "local_position.h"

LocalPosition::LocalPosition()
{
    this->posX = 0.0;
    this->posY = 0.0;
    this->posZ = 0.0;
}

LocalPosition::LocalPosition(const double &x, const double &y, const double &z)
{
    this->posX = x;
    this->posY = y;
    this->posZ = z;
}

LocalPosition::LocalPosition(const LocalPosition &copyObj)
{
    this->posX = copyObj.getX();
    this->posY = copyObj.getY();
    this->posZ = copyObj.getZ();
}

void LocalPosition::setPosition(const double &x, const double &y, const double &z)
{
    this->posX = x;
    this->posY = y;
    this->posZ = z;
}

double LocalPosition::bearingBetween(const LocalPosition &position)
{
    return 0.0;
}

double LocalPosition::initialBearing(const LocalPosition &position){
    return 0.0;
    //return (bearingBetween(position) + 360.0) % 360.0;
}

double LocalPosition::finalBearing(const LocalPosition &position){
    return 0.0;
    //return (bearingBetween(position) + 180.0) % 360.0;
}

double LocalPosition::distanceBetween(const LocalPosition &position)
{
    double deltaX = position.getX() - this->getX();
    double deltaY = position.getY() - this->getY();
    double deltaZ = position.getZ() - this->getZ();

    return sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
}

double LocalPosition::getZ() const
{
    return posZ;
}

double LocalPosition::getX() const
{
    return posX;
}

double LocalPosition::getY() const
{
    return posY;
}

double LocalPosition::convertDegreesToRadians(const double &degrees)
{
    double pi = 3.14159265358979323846;
    double radians = degrees * (pi/180.0);
    return radians;
}

double LocalPosition::convertRadiansToDegrees(const double &radians)
{
    double pi = 3.14159265358979323846;
    double degrees = radians * (180.0/pi);
    return degrees;
}
