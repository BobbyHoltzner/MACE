#ifndef SPATIAL_ABSTRACT_POSITION_H
#define SPATIAL_ABSTRACT_POSITION_H

#include <iostream>
#include <sstream>

#include "data_generic_state_item/base_3d_position.h"

namespace CommandItem {

//!
//! \brief The SpatialAbstractPosition class
//!
class SpatialAbstractPosition
{
public:
    SpatialAbstractPosition()
    {
        position = new DataState::Base3DPosition();
    }

    virtual ~SpatialAbstractPosition()
    {
        if(position)
        {
            delete position;
            position = NULL;
        }
    }

    SpatialAbstractPosition(const SpatialAbstractPosition &obj)
    {
        this->position = obj.position;
    }


public:
    //!
    //! \brief operator =
    //! \param rhs
    //!
    SpatialAbstractPosition& operator = (const SpatialAbstractPosition &rhs)
    {
        this->position = rhs.position;
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const SpatialAbstractPosition &rhs) {
        if(this->position != rhs.position)
        {
            return false;
        }
        return true;
    }

    //!
    //! \brief operator !=
    //! \param rhs
    //! \return
    //!
    bool operator != (const SpatialAbstractPosition &rhs) {
        return !(*this == rhs);
    }

    friend std::ostream& operator<<(std::ostream& os, const SpatialAbstractPosition& t);

public:
    //!
    //! \brief position
    //!
    DataState::Base3DPosition *position;
};

} //end of namespace MissionItem

#endif // SPATIAL_ABSTRACT_POSITION_H
