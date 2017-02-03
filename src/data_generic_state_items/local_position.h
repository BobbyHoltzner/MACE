#ifndef DATASTATE_LOCALPOSITION_H
#define DATASTATE_LOCALPOSITION_H

#include "data/coordinate_frame.h"
#include "position.h"

namespace DataState
{

class LocalPosition : public Position
{
public:

    LocalPosition();

    LocalPosition(const Data::CoordinateFrame &frame);

    LocalPosition(const double &x, const double &y, const double &z);

    LocalPosition(const Data::CoordinateFrame &frame, const double &x, const double &y, const double &z);

public:
    virtual Data::CoordinateFrame getCoordinateFrame();
    virtual Data::PositionalFrame getPositionFrame();

public:

    double distanceBetween(const LocalPosition &position);

    double bearingBetween(const LocalPosition &position);

    double finalBearing(const LocalPosition &postion);

    double initialBearing(const LocalPosition &postion);

public:
    double x;
    double y;
    double z;
};

}

#endif //DATASTATE_LOCALPOSITION_H
