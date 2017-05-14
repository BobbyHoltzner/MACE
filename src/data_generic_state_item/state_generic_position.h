#ifndef STATE_GENERIC_POSITION_H
#define STATE_GENERIC_POSITION_H

#include "data/positional_coordinate_frame.h"
#include "data/coordinate_frame.h"

using namespace Data;

namespace DataState {
class StateGenericPosition
{
public:

    void setCoordinateFrame(const CoordinateFrameType &coordinateFrame){
        this->m_CoordinateFrame = coordinateFrame;
    }

    CoordinateFrameType getCoordinateFrame() const {
        return m_CoordinateFrame;
    }

public:
    void operator = (const StateGenericPosition &rhs)
    {
        this->m_CoordinateFrame = rhs.m_CoordinateFrame;
    }

    bool operator == (const StateGenericPosition &rhs) {
        if(this->m_CoordinateFrame != rhs.m_CoordinateFrame){
            return false;
        }
        return true;
    }

    bool operator != (const StateGenericPosition &rhs) {
        return !(*this == rhs);
    }
protected:
    CoordinateFrameType m_CoordinateFrame;
};

}

#endif // STATE_GENERIC_POSITION_H
