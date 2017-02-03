#include "position.h"

namespace DataVehicleGeneric {

Position::Position()
{

}

Position::Position(const Data::PositionalFrame &frame)
{
    m_PositionFrame = frame;
}

Position::Position(const Data::PositionalFrame &positionFrame, const Data::CoordinateFrame &coordinateFrame)
{
    m_PositionFrame = positionFrame;
    m_CoordinateFrame = coordinateFrame;
}

Position::~Position()
{

}

Data::PositionalFrame Position::getPositionFrame()
{
    return m_PositionFrame;
}

/**
 * @brief Position::convertDegreesToRadians
 * @param degrees
 * @return
 */
double Position::convertDegreesToRadians(const double &degrees)
{
    double pi = 3.14159265358979323846;
    double radians = degrees * (pi/180.0);
    return radians;
}

/**
 * @brief Position::convertRadiansToDegrees
 * @param radians
 * @return
 */
double Position::convertRadiansToDegrees(const double &radians)
{
    double pi = 3.14159265358979323846;
    double degrees = radians * (180.0/pi);
    return degrees;
}

}
