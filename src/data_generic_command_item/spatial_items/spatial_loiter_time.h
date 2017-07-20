#ifndef SPATIAL_LOITER_TIME_H
#define SPATIAL_LOITER_TIME_H

#include <iostream>

#include "data/command_item_type.h"
#include "data/loiter_direction.h"

#include "data_generic_command_item/abstract_command_item.h"

#include "data_generic_state_item/state_global_position.h"
#include "data_generic_state_item/state_local_position.h"

namespace CommandItem {

template <class T>
class SpatialLoiter_Time : public AbstractCommandItem
{
public:
    virtual Data::CommandItemType getCommandType()const;

    virtual std::string getDescription()const;

    virtual bool hasSpatialInfluence()const;

public:
    SpatialLoiter_Time();
    SpatialLoiter_Time(const SpatialLoiter_Time &obj);
    SpatialLoiter_Time(const int &systemOrigin, const int &systemTarget = 0);

public:
    void operator = (const SpatialLoiter_Time &rhs)
    {
        AbstractCommandItem::operator =(rhs);
        this->position = rhs.position;
        this->direction = rhs.direction;
        this->radius = rhs.radius;
        this->duration = rhs.duration;
    }

    bool operator == (const SpatialLoiter_Time &rhs) {
        if(!AbstractCommandItem::operator ==(rhs))
        {
            return false;
        }
        if(this->position != rhs.position){
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
        if(this->duration != rhs.duration)
        {
            return false;
        }
        return true;
    }

    bool operator != (const SpatialLoiter_Time &rhs) {
        return !(*this == rhs);
    }

public:
    T position;
    Data::LoiterDirection direction;
    double radius;
    double duration;
};

} //end of namespace MissionItem

#endif // SPATIAL_LOITER_TIME_H
