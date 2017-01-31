#ifndef POSITION_H
#define POSITION_H

#include "data/positional_coordinate_frame.h"
#include "coordinate_frame.h"

namespace DataVehicleGeneric {

class Position
{
public:
    Position();

    Position(const Data::PositionalFrame &positionFrame);

    Position(const Data::PositionalFrame &positionFrame, const CoordinateFrame &coordinateFrame);

    virtual ~Position();

    Data::PositionalFrame  getPositionFrame();

protected:
    static double convertDegreesToRadians(const double &degrees);

    static double convertRadiansToDegrees(const double &radians);

protected:
    Data::PositionalFrame m_PositionFrame = Data::PositionalFrame::GLOBAL;
    CoordinateFrame m_CoordinateFrame = CoordinateFrame::NED;
};

} //end of namespace DataVehicleGeneric
#endif // POSITION_H
