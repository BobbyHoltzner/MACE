#include "local_position.h"

#include <math.h>

namespace DataState
{

LocalPosition::LocalPosition()
{

}

LocalPosition::LocalPosition(const Data::CoordinateFrame &frame)
{

}

LocalPosition::LocalPosition(const double &x, const double &y, const double &z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

LocalPosition::LocalPosition(const Data::CoordinateFrame &frame, const double &x, const double &y, const double &z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}

Data::CoordinateFrame LocalPosition::getCoordinateFrame()
{
    return m_CoordinateFrame;
}

Data::PositionalFrame LocalPosition::getPositionFrame()
{
    return m_PositionFrame;
}

double LocalPosition::bearingBetween(const LocalPosition &position)
{
    throw std::runtime_error("Not Implimented");
    return 0.0;
}

double LocalPosition::initialBearing(const LocalPosition &position){
    throw std::runtime_error("Not Implimented");
    return 0.0;
    //return (bearingBetween(position) + 360.0) % 360.0;
}

double LocalPosition::finalBearing(const LocalPosition &position){
    throw std::runtime_error("Not Implimented");
    return 0.0;
    //return (bearingBetween(position) + 180.0) % 360.0;
}

double LocalPosition::distanceBetween(const LocalPosition &position)
{
    double deltaX = position.x - this->x;
    double deltaY = position.y - this->y;
    double deltaZ = position.z - this->z;

    return sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
}

}
