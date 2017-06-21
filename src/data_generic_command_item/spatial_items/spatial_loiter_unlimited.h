#ifndef SPATIAL_LOITER_UNLIMITED_H
#define SPATIAL_LOITER_UNLIMITED_H

#include <iostream>

#include "data/command_item_type.h"
#include "data/loiter_direction.h"

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_state_item/state_generic_position.h"

namespace CommandItem {

class SpatialLoiter_Unlimited : public AbstractCommandItem
{
public:
    virtual Data::CommandItemType getCommandType()const;

    virtual std::string getDescription()const;

    virtual bool hasSpatialInfluence()const;

public:
    SpatialLoiter_Unlimited();
    SpatialLoiter_Unlimited(const SpatialLoiter_Unlimited &obj);
    SpatialLoiter_Unlimited(const int &originatingSystem, const int &systemTarget = 0);

public:
    void operator = (const SpatialLoiter_Unlimited &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this.position = rhs.position;
        this->direction = rhs.direction;
        this->radius = rhs.radius;
    }

    bool operator == (const SpatialLoiter_Unlimited &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(this->position != rhs.position)
        {
            return false;
        }
        if(this->direction != rhs.direction)
        {
            return false;
        }
        if(this->radius != rhs.radius)
        {
            return false;
        }
        return true;
    }

    bool operator != (const SpatialLoiter_Unlimited &rhs) {
        return !(*this == rhs);
    }

public:
    DataState::StateGenericPosition position;
    Data::LoiterDirection direction;
    double radius;
};

}

#endif // SPATIAL_LOITER_UNLIMITED_H
