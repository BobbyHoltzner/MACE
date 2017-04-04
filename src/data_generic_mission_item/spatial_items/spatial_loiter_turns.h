#ifndef SPATIAL_LOITER_TURNS_H
#define SPATIAL_LOITER_TURNS_H

#include <iostream>

#include "data/loiter_direction.h"

#include "data_generic_mission_item/abstract_mission_item.h"
#include "data_generic_mission_item/mission_item_types.h"

#include "data_generic_state_item/state_global_position.h"
#include "data_generic_state_item/state_local_position.h"

namespace MissionItem {

template <class T>
class SpatialLoiter_Turns : public AbstractMissionItem
{
public:
    virtual MissionItemType getMissionType()const;

    virtual std::string getDescription()const;

    virtual bool hasSpatialMissionInfluence()const;

public:
    SpatialLoiter_Turns();

public:
    void operator = (const SpatialLoiter_Turns &rhs)
    {
        AbstractMissionItem::operator =(rhs);
        this->position = rhs.position;
        this->direction = rhs.direction;
        this->radius = rhs.radius;
        this->turns = rhs.turns;
    }

    bool operator == (const SpatialLoiter_Turns &rhs) {
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
        if(this->turns != rhs.turns)
        {
            return false;
        }
        return true;
    }

    bool operator != (const SpatialLoiter_Turns &rhs) {
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out);

public:
    T position;
    Data::LoiterDirection direction;
    double radius;
    double turns;
};

} //end of namespace MissionItem

#endif // SPATIAL_LOITER_TURNS_H
