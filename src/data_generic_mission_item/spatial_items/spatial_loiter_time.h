#ifndef SPATIAL_LOITER_TIME_H
#define SPATIAL_LOITER_TIME_H

#include <iostream>

#include "data/mission_item_type.h"
#include "data/loiter_direction.h"

#include "data_generic_mission_item/abstract_mission_item.h"
#include "data_generic_state_item/state_global_position.h"
#include "data_generic_state_item/state_local_position.h"



namespace MissionItem {

template <class T>
class SpatialLoiter_Time : public AbstractMissionItem
{
public:
    virtual Data::MissionItemType getMissionType()const;

    virtual std::string getDescription()const;

    virtual bool hasSpatialMissionInfluence()const;

public:
    SpatialLoiter_Time();

public:
    void operator = (const SpatialLoiter_Time &rhs)
    {
        AbstractMissionItem::operator =(rhs);
        this->position = rhs.position;
        this->direction = rhs.direction;
        this->radius = rhs.radius;
        this->duration = rhs.duration;
    }

    bool operator == (const SpatialLoiter_Time &rhs) {
        if(!AbstractMissionItem::operator ==(rhs))
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

    std::ostream& operator<<(std::ostream &out);

public:
    T position;
    Data::LoiterDirection direction;
    double radius;
    double duration;
};

} //end of namespace MissionItem

#endif // SPATIAL_LOITER_TIME_H
