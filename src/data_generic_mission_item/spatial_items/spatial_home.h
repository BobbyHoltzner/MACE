#ifndef SPATIAL_HOME_H
#define SPATIAL_HOME_H

#include <iostream>

#include "data/mission_item_type.h"
#include "data_generic_mission_item/abstract_mission_item.h"
#include "data_generic_state_item/state_global_position.h"
#include "data_generic_state_item/state_local_position.h"

using namespace Data;

namespace MissionItem {

class SpatialHome : public AbstractMissionItem
{
public:
    SpatialHome();
    SpatialHome(const SpatialHome &spatialHome);

public:
    virtual MissionItemType getMissionType()const;

    virtual std::string getDescription()const;

    virtual bool hasSpatialMissionInfluence()const;


public:
    void operator = (const SpatialHome &rhs)
    {
        AbstractMissionItem::operator =(rhs);
        this->position = rhs.position;
    }

    bool operator == (const SpatialHome &rhs) {
        if(!AbstractMissionItem::operator ==(rhs))
        {
            return false;
        }
        if(this->position != rhs.position){
            return false;
        }
        return true;
    }

    bool operator != (const SpatialHome &rhs) {
        return !(*this == rhs);
    }

    std::ostream& operator<<(std::ostream &out)
    {
        out<<"Spatial Home(SystemID: "<<m_VehicleID<<", Latitude: "<<position.latitude<<", Longitude: "<<position.longitude<<", Altitude: "<<position.altitude<<")";
        return out;
    }

public:
    DataState::StateGlobalPosition position;
};

} //end of namespace MissionItem

#endif // SPATIAL_HOME_H
