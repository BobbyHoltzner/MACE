#ifndef STATE_GENERIC_POSITION_H
#define STATE_GENERIC_POSITION_H

#include "data/positional_coordinate_frame.h"
#include "data/coordinate_frame.h"

namespace DataState {
class StateGenericPosition
{
public:

    void setCoordinateFrame(const Data::CoordinateFrame &coordinateFrame){
        this->m_CoordinateFrame = coordinateFrame;
    }

    Data::PositionalFrame getPositionFrame() const {
        return m_PositionFrame;
    }

    Data::CoordinateFrame getCoordinateFrame() const {
        return m_CoordinateFrame;
    }

public:
    void operator = (const StateGenericPosition &rhs)
    {
        this->m_PositionFrame = rhs.m_PositionFrame;
        this->m_CoordinateFrame = rhs.m_CoordinateFrame;
    }

    bool operator == (const StateGenericPosition &rhs) {
        if(this->m_PositionFrame != rhs.m_PositionFrame){
            return false;
        }
        if(this->m_CoordinateFrame != rhs.m_CoordinateFrame){
            return false;
        }
        return true;
    }

    bool operator != (const StateGenericPosition &rhs) {
        return !(*this == rhs);
    }
protected:
    Data::PositionalFrame m_PositionFrame;
    Data::CoordinateFrame m_CoordinateFrame;
};

}

#endif // STATE_GENERIC_POSITION_H
