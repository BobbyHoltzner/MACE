#ifndef STATE_GLOBAL_POSITION_H
#define STATE_GLOBAL_POSITION_H

#include <iostream>

#include <math.h>

#include "common/common.h"

#include "data/coordinate_frame.h"

#include "state_generic_position.h"

#include <Eigen/Dense>

namespace DataState {

class StateGlobalPosition : public StateGenericPosition<StateGlobalPosition>
{
public:

    StateGlobalPosition();

    StateGlobalPosition(const StateGlobalPosition &globalPosition);

    StateGlobalPosition(const Data::CoordinateFrameType &frame);

    StateGlobalPosition(const float &latitude, const float &longitude, const float &altitude);

    StateGlobalPosition(const Data::CoordinateFrameType &frame, const double &latitude, const double &longitude, const double &altitude);


public:
    void setPosition(const double &latitude, const double &longitude, const double &altitude);
    void setLatitude(const double &value);
    void setLongitude(const double &value);
    void setAltitude(const double &value);

    double getLatitude() const;
    double getLongitude() const;
    double getAltitude() const;

public:
    static double convertDegreesToRadians(const double &degrees);

    static double convertRadiansToDegrees(const double &radians);


public:
    virtual double deltaAltitude(const StateGlobalPosition &position) const;
    virtual double distanceBetween2D(const StateGlobalPosition &position) const;
    virtual double distanceBetween3D(const StateGlobalPosition &position) const;

public:
    virtual double finalBearing(const StateGlobalPosition &postion) const;
    virtual double initialBearing(const StateGlobalPosition &postion) const;
    virtual double bearingBetween(const StateGlobalPosition &position) const;

    virtual StateGlobalPosition NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag) const;
    virtual void translationTransformation(const StateGlobalPosition &position, Eigen::Vector3f &transVec);

public:
    void operator = (const StateGlobalPosition &rhs)
    {
        StateGenericPosition::operator =(rhs);
    }

    bool operator == (const StateGlobalPosition &rhs) {

        if(!StateGenericPosition::operator ==(rhs)){
            return false;
        }
        return true;
    }

    bool operator != (const StateGlobalPosition &rhs) {
        return !(*this == rhs);
    }
};

} //end of namespace DataState

#endif // STATE_GLOBAL_POSITION_H
