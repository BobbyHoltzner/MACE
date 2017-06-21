#ifndef SPATIAL_LOITER_TURNS_H
#define SPATIAL_LOITER_TURNS_H

#include <iostream>

#include "data/loiter_direction.h"
#include "data/command_item_type.h"

#include "data_generic_command_item/abstract_command_item.h"
#include "data_generic_state_item/state_generic_position.h"

namespace CommandItem {

class SpatialLoiter_Turns : public AbstractCommandItem
{
public:
    virtual Data::CommandItemType getCommandType()const;

    virtual std::string getDescription()const;

    virtual bool hasSpatialInfluence()const;

public:
    SpatialLoiter_Turns();
    SpatialLoiter_Turns(const SpatialLoiter_Turns &obj);
    SpatialLoiter_Turns(const int &originatingSystem, const int &systemTarget = 0);

public:
    void operator = (const SpatialLoiter_Turns &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this->position = rhs.position;
        this->direction = rhs.direction;
        this->radius = rhs.radius;
        this->turns = rhs.turns;
    }

    bool operator == (const SpatialLoiter_Turns &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(!DataState::StateGenericPosition<T>::operator ==(rhs))
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
        if(this->turns != rhs.turns)
        {
            return false;
        }
        return true;
    }

    bool operator != (const SpatialLoiter_Turns &rhs) {
        return !(*this == rhs);
    }

public:
    DataState::StateGenericPosition position;
    Data::LoiterDirection direction;
    double radius;
    double turns;
};

} //end of namespace MissionItem

#endif // SPATIAL_LOITER_TURNS_H
