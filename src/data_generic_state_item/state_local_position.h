#ifndef STATE_LOCAL_POSITION_H
#define STATE_LOCAL_POSITION_H

#include <iostream>
#include "common/common.h"

#include "data/coordinate_frame.h"

#include "state_generic_position.h"

using namespace Data;

namespace DataState {

class StateLocalPosition : public StateGenericPosition
{
public:
    StateLocalPosition();

    StateLocalPosition(const StateLocalPosition &localPosition);

    StateLocalPosition(const CoordinateFrameType &frame);

    StateLocalPosition(const double &x, const double &y, const double &z);

    StateLocalPosition(const CoordinateFrameType &frame, const double &x, const double &y, const double &z);

public:

    double distanceBetween(const StateLocalPosition &position);

    double bearingBetween(const StateLocalPosition &position);

    double finalBearing(const StateLocalPosition &postion);

    double initialBearing(const StateLocalPosition &postion);

    bool essentiallyEquivalent_Percentage(const StateLocalPosition &rhs, const double &percentage);

    bool essentiallyEquivalent_Distance(const StateLocalPosition &rhs, const double &distance);


public:
    void operator = (const StateLocalPosition &rhs)
    {
        StateGenericPosition::operator =(rhs);
        this->x = rhs.x;
        this->y = rhs.y;
        this->z = rhs.z;
    }

    bool operator == (const StateLocalPosition &rhs) {
        if(!StateGenericPosition::operator ==(rhs)){
            return false;
        }
        if(this->x != rhs.x){
            return false;
        }
        if(this->y != rhs.y){
            return false;
        }
        if(this->z != rhs.z){
            return false;
        }
        return true;
    }

    bool operator != (const StateLocalPosition &rhs) {
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out)
    {
        out<<"Local Position( X: "<<x<<", Y: "<<y<<", Z: "<<z<<")";
        return out;
    }

public:
    CoordinateFrameType m_CoordinateFrame;

    double x;
    double y;
    double z;
};

} //end of namespace DataState

#endif // STATE_LOCAL_POSITION_H
