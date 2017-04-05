#ifndef STATE_GLOBAL_POSITION_EX_H
#define STATE_GLOBAL_POSITION_EX_H

#include "data_generic_state_item/state_global_position.h"

namespace DataState{

class StateGlobalPositionEx : public StateGlobalPosition
{
public:
    StateGlobalPositionEx();

    StateGlobalPositionEx(const StateGlobalPosition &globalPosition, const double &globalHeading);

public:
    void operator = (const StateGlobalPositionEx &rhs)
    {
        StateGlobalPosition::operator =(rhs);
        this->heading = rhs.heading;
    }

    bool operator == (const StateGlobalPositionEx &rhs) {
        if(!StateGlobalPosition::operator ==(rhs)){
            return false;
        }
        if(this->heading != rhs.heading){
            return false;
        }
        return true;
    }

    bool operator != (const StateGlobalPositionEx &rhs) {
        return !(*this == rhs);
    }

public:
    double heading;

};

} //end of namespace DataState
#endif // STATE_GLOBAL_POSITION_EX_H
