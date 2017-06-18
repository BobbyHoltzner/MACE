#ifndef STATE_LOCAL_POSITION_H
#define STATE_LOCAL_POSITION_H

#include <iostream>
#include "common/common.h"

#include "data/coordinate_frame.h"

#include "state_generic_position.h"



namespace DataState {

class StateLocalPosition : public StateGenericPosition<StateLocalPosition>
{
public:
    StateLocalPosition();

    StateLocalPosition(const StateLocalPosition &localPosition);

    StateLocalPosition(const Data::CoordinateFrameType &frame);

    StateLocalPosition(const double &x, const double &y, const double &z);

    StateLocalPosition(const Data::CoordinateFrameType &frame, const double &x, const double &y, const double &z);

public:
    void setPosition(const double &posX, const double &posY, const double &posZ);
    void setPositionX(const double &value);
    void setPositionY(const double &value);
    void setPositionZ(const double &value);

    double getPositionX() const;
    double getPositionY() const;
    double getPositionZ() const;

public:
    virtual double deltaAltitude(const StateLocalPosition &position) const;
    virtual double distanceBetween2D(const StateLocalPosition &position) const;
    virtual double distanceBetween3D(const StateLocalPosition &position) const;

public:
    virtual double finalBearing(const StateLocalPosition &position) const;
    virtual double initialBearing(const StateLocalPosition &position) const;
    virtual double bearingBetween(const StateLocalPosition &position) const;
    virtual StateLocalPosition NewPositionFromHeadingBearing(const double &distance, const double &bearing, const bool &degreesFlag) const;
    virtual void translationTransformation(const StateLocalPosition &position, Eigen::Vector3f &transVec);

public:
    bool essentiallyEquivalent_Percentage(const StateLocalPosition &rhs, const double &percentage);
    bool essentiallyEquivalent_Distance(const StateLocalPosition &rhs, const double &distance);

public:
    void operator = (const StateLocalPosition &rhs)
    {
        StateGenericPosition::operator =(rhs);
    }

    bool operator == (const StateLocalPosition &rhs) {
        if(!StateGenericPosition::operator ==(rhs)){
            return false;
        }
        return true;
    }

    bool operator != (const StateLocalPosition &rhs) {
        return !(*this == rhs);
    }
};

} //end of namespace DataState

#endif // STATE_LOCAL_POSITION_H
