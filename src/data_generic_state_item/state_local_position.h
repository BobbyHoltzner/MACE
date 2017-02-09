#ifndef STATE_LOCAL_POSITION_H
#define STATE_LOCAL_POSITION_H

#include "data/positional_coordinate_frame.h"
#include "data/coordinate_frame.h"

namespace DataState {

class StateLocalPosition
{
public:
    StateLocalPosition();

    StateLocalPosition(const Data::CoordinateFrame &frame);

    StateLocalPosition(const double &x, const double &y, const double &z);

    StateLocalPosition(const Data::CoordinateFrame &frame, const double &x, const double &y, const double &z);

public:
    virtual Data::CoordinateFrame getCoordinateFrame() const;
    virtual Data::PositionalFrame getPositionFrame() const;

public:

    double distanceBetween(const StateLocalPosition &position);

    double bearingBetween(const StateLocalPosition &position);

    double finalBearing(const StateLocalPosition &postion);

    double initialBearing(const StateLocalPosition &postion);

public:
    Data::PositionalFrame m_PositionFrame;
    Data::CoordinateFrame m_CoordinateFrame;

    double x;
    double y;
    double z;
};

} //end of namespace DataState

#endif // STATE_LOCAL_POSITION_H
