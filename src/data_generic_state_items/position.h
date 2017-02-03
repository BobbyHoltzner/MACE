#ifndef POSITION_H
#define POSITION_H

#include "data/positional_coordinate_frame.h"
#include "data/coordinate_frame.h"

namespace DataState {

class Position
{
public:
    virtual Data::PositionalFrame getPositionFrame() const = 0;

    virtual Data::CoordinateFrame getCoordinateFrame() const = 0;

protected:
    static double convertDegreesToRadians(const double &degrees);

    static double convertRadiansToDegrees(const double &radians);

protected:
    Data::PositionalFrame m_PositionFrame;
    Data::CoordinateFrame m_CoordinateFrame;
};

} //end of namespace DataVehicleGeneric
#endif // POSITION_H
