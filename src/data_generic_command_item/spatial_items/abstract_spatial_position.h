#ifndef ABSTRACT_SPATIAL_POSITION_H
#define ABSTRACT_SPATIAL_POSITION_H

#include <iostream>
#include <sstream>

#include "data_generic_state_item/base_3d_position.h"

using namespace DataState;

namespace CommandItem {

//!
//! \brief The AbstractSpatialPosition class
//!
class AbstractSpatialPosition
{
public:
    AbstractSpatialPosition()
    {
        position = new Base3DPosition();
    }

    ~AbstractSpatialPosition()
    {
        if(position)
        {
            delete position;
            position = NULL;
        }
    }

    AbstractSpatialPosition(const AbstractSpatialPosition &copy)
    {
        //position is a pointer, so we need to deep copy it if it is non-null
        //allocate the memory
        position = new Base3DPosition();
        if(copy.position)
        {
            //copy the contents
            *position = *copy.position;
        }
    }

    void setPosition(const Base3DPosition &pos)
    {
        if(this->position != NULL)
        {
            delete this->position;
        }
        position = new Base3DPosition (pos);
    }

    Base3DPosition getPosition()
    {
        return *this->position;
    }

    const Base3DPosition getPosition() const
    {
        return *this->position;
    }


public:
    //!
    //! \brief operator =
    //! \param rhs
    //!
    AbstractSpatialPosition& operator = (const AbstractSpatialPosition &rhs)
    {
        //self-assignment gaurd
        if(this == &rhs)
            return *this;

        if(rhs.position)
        {
            *this->position = *rhs.position;
        }else{ //test is null
            position = new Base3DPosition();
        }
        return *this;
    }

    //!
    //! \brief operator ==
    //! \param rhs
    //! \return
    //!
    bool operator == (const AbstractSpatialPosition &rhs) {
        if(*this->position != *rhs.position)
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
    bool operator != (const AbstractSpatialPosition &rhs) {
        return !(*this == rhs);
    }

    friend std::ostream& operator<<(std::ostream& os, const AbstractSpatialPosition& t);

public:

    //!
    //! \brief position
    //!
    Base3DPosition *position;
};

} //end of namespace MissionItem
#endif // ABSTRACT_SPATIAL_POSITION_H
