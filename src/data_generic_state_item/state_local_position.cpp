#include "state_local_position.h"
#include <math.h>

using namespace DataState;

StateLocalPosition::StateLocalPosition():
    StateGenericPosition(Data::CoordinateFrameType::CF_LOCAL_ENU)
{

}

StateLocalPosition::StateLocalPosition(const StateLocalPosition &localPosition)
{
    this->operator =(localPosition);
}

StateLocalPosition::StateLocalPosition(const Data::CoordinateFrameType &frame):
    StateGenericPosition(frame)
{

}

StateLocalPosition::StateLocalPosition(const double &posX, const double &posY, const double &posZ):
    StateGenericPosition(Data::CoordinateFrameType::CF_LOCAL_ENU,posX,posY,posZ)
{

}

StateLocalPosition::StateLocalPosition(const Data::CoordinateFrameType &frame, const double &posX, const double &posY, const double &posZ):
    StateGenericPosition(frame,posX,posY,posZ)
{

}

void StateLocalPosition::setPosition(const double &posX, const double &posY, const double &posZ)
{
    this->setX(posX);
    this->setY(posY);
    this->setZ(posZ);
}

void StateLocalPosition::setPositionX(const double &value)
{
    this->setX(value);
}

void StateLocalPosition::setPositionY(const double &value)
{
    this->setY(value);
}

void StateLocalPosition::setPositionZ(const double &value)
{
    this->setZ(value);
}

double StateLocalPosition::getPositionX() const
{
    return getX();
}
double StateLocalPosition::getPositionY() const
{
    return this->getY();
}
double StateLocalPosition::getPositionZ() const
{
    return this->getZ();
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

    double StateLocalPosition::deltaAltitude(const StateLocalPosition &position) const
    {
        double deltaZ = position.z - this->z;
        return deltaZ;
    }
    double StateLocalPosition::distanceBetween2D(const StateLocalPosition &position) const
    {
        double deltaX = position.x - this->x;
        double deltaY = position.y - this->y;

        return sqrt(deltaX * deltaX + deltaY * deltaY);
    }
    double StateLocalPosition::distanceBetween3D(const StateLocalPosition &position) const
    {
        double deltaX = position.x - this->x;
        double deltaY = position.y - this->y;
        double deltaZ = position.z - this->z;

        return sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
    }

    double StateLocalPosition::finalBearing(const StateLocalPosition &position) const
    {
        throw std::runtime_error("Not Implimented");
        UNUSED(position);
        return 0.0;
    }
    double StateLocalPosition::initialBearing(const StateLocalPosition &position) const
    {
        throw std::runtime_error("Not Implimented");
        UNUSED(position);
        return 0.0;
    }
    double StateLocalPosition::bearingBetween(const StateLocalPosition &position) const
    {
        UNUSED(position);
        throw std::runtime_error("Not Implimented");
        return 0.0;
    }
    StateLocalPosition StateLocalPosition::NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag) const
    {
        UNUSED(distance);
        UNUSED(bearing);
        UNUSED(degreesFlag);
        throw std::runtime_error("Not Implimented");

        StateLocalPosition newTemp;
        return newTemp;

    }
    void StateLocalPosition::translationTransformation(const StateLocalPosition &position, Eigen::Vector3f &transVec)
    {
        UNUSED(position);
        UNUSED(transVec);
        throw std::runtime_error("Not Implimented");
    }


