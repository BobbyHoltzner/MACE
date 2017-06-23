#ifndef SPATIAL_LAND_H
#define SPATIAL_LAND_H

#include <iostream>

#include "data/command_item_type.h"

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_state_item/state_generic_position.h"

namespace CommandItem {

class SpatialLand : public AbstractCommandItem
{
public:
    virtual Data::CommandItemType getCommandType()const;

    virtual std::string getDescription()const;

    virtual bool hasSpatialInfluence()const;

public:
    SpatialLand();
    SpatialLand(const SpatialLand &obj);
    SpatialLand(const int &systemOrigin, const int &systemTarget = 0);

    bool getLandFlag() const;
    void setLandFlag(const bool &landFlag);

public:
    void operator = (const SpatialLand &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this->position = rhs.position;
    }

    bool operator == (const SpatialLand &rhs) {
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

    bool operator != (const SpatialLand &rhs) {
        return !(*this == rhs);
    }

public:
    DataState::StateGenericPosition position;

};

} //end of namespace MissionItem

#endif // SPATIAL_LAND_H
