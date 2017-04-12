#include "state_local_position.h"
#include <math.h>

using namespace DataState;

StateLocalPosition::StateLocalPosition():
    x(0.0),y(0.0),z(0.0)
{
    m_PositionFrame = Data::PositionalFrame::LOCAL;
    m_CoordinateFrame = Data::CoordinateFrame::NED;
}

StateLocalPosition::StateLocalPosition(const StateLocalPosition &localPosition)
{
    this->x = localPosition.x;
    this->y = localPosition.y;
    this->z = localPosition.z;
}

StateLocalPosition::StateLocalPosition(const Data::CoordinateFrame &frame)
{
    m_PositionFrame = Data::PositionalFrame::LOCAL;
    m_CoordinateFrame = frame;
}

StateLocalPosition::StateLocalPosition(const double &x, const double &y, const double &z)
{
    m_PositionFrame = Data::PositionalFrame::LOCAL;
    m_CoordinateFrame = Data::CoordinateFrame::NED;

    this->x = x;
    this->y = y;
    this->z = z;
}

StateLocalPosition::StateLocalPosition(const Data::CoordinateFrame &frame, const double &x, const double &y, const double &z)
{
    m_PositionFrame = Data::PositionalFrame::LOCAL;
    m_CoordinateFrame = frame;

    this->x = x;
    this->y = y;
    this->z = z;
}

bool StateLocalPosition::essentiallyEquivalent_Percentage(const StateLocalPosition &rhs, const double &percentage)
{
   double changeX = (fabs(this->x - rhs.x)/fabs(this->x)) * 100.0;
   double changeY = (fabs(this->y - rhs.y)/fabs(this->y)) * 100.0;
   double changeZ = (fabs(this->z - rhs.z)/fabs(this->z)) * 100.0;

   if(changeX > percentage)
       return false;
   if(changeY > percentage)
       return false;
   if(changeZ > percentage)
       return false;

    return true;
}

bool StateLocalPosition::essentiallyEquivalent_Distance(const StateLocalPosition &rhs, const double &distance)
{
   double changeX = fabs(this->x - rhs.x);
   double changeY = fabs(this->y - rhs.y);
   double changeZ = fabs(this->z - rhs.z);

   if(changeX > distance)
       return false;
   if(changeY > distance)
       return false;
   if(changeZ > distance)
       return false;

    return true;
}

double StateLocalPosition::bearingBetween(const StateLocalPosition &position)
{
    UNUSED(position);
    throw std::runtime_error("Not Implimented");
    return 0.0;
}

double StateLocalPosition::initialBearing(const StateLocalPosition &position)
{
    throw std::runtime_error("Not Implimented");
    UNUSED(position);
    return 0.0;
    //return (bearingBetween(position) + 360.0) % 360.0;
}

double StateLocalPosition::finalBearing(const StateLocalPosition &position)
{
    throw std::runtime_error("Not Implimented");
    UNUSED(position);
    return 0.0;
    //return (bearingBetween(position) + 180.0) % 360.0;
}

double StateLocalPosition::distanceBetween(const StateLocalPosition &position)
{
    double deltaX = position.x - this->x;
    double deltaY = position.y - this->y;
    double deltaZ = position.z - this->z;

    return sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
}

