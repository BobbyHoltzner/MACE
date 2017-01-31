#ifndef POSITION_H
#define POSITION_H

#include "coordinate_frame.h"

namespace DataVehicleGeneric {

enum class PositionFrame{
    LOCAL,
    GLOBAL
};


class Position
{
public:
    Position();

    Position(const PositionFrame &positionFrame);

    Position(const PositionFrame &positionFrame, const CoordinateFrame &coordinateFrame);

    virtual ~Position();

    PositionFrame getPositionFrame();

protected:
    static double convertDegreesToRadians(const double &degrees);

    static double convertRadiansToDegrees(const double &radians);

protected:
    PositionFrame m_PositionFrame = PositionFrame::GLOBAL;
    CoordinateFrame m_CoordinateFrame = CoordinateFrame::NED;
};

} //end of namespace DataVehicleGeneric
#endif // POSITION_H
