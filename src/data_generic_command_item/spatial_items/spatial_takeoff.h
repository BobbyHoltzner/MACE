#ifndef SPATIAL_TAKEOFF_H
#define SPATIAL_TAKEOFF_H

#include <iostream>

#include "data/command_item_type.h"

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_state_item/base_3d_position.h"

namespace CommandItem {

class SpatialTakeoff : public AbstractCommandItem
{

public:
    virtual Data::CommandItemType getCommandType()const;

    virtual std::string getDescription()const;

    virtual bool hasSpatialInfluence()const;

public:
    SpatialTakeoff();
    SpatialTakeoff(const SpatialTakeoff &obj);
    SpatialTakeoff(const int &systemOrigin, const int &systemTarget = 0);

public:
    void operator = (const SpatialTakeoff &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this->position = rhs.position;
    }

    bool operator == (const SpatialTakeoff &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(this->position != rhs.position)
        {
            return false;
        }
        return true;
    }

    bool operator != (const SpatialTakeoff &rhs) {
        return !(*this == rhs);
    }

public:
    DataState::Base3DPosition position;
};

} //end of namespace MissionItem

#endif // SPATIAL_TAKEOFF_H
