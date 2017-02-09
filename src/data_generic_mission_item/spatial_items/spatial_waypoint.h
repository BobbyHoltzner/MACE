#ifndef SPATIAL_WAYPOINT_H
#define SPATIAL_WAYPOINT_H

#include "data_generic_mission_item/abstract_mission_item.h"
#include "data_generic_mission_item/mission_item_types.h"

#include "data_generic_state_item/state_global_position.h"
#include "data_generic_state_item/state_local_position.h"

namespace MissionItem {

template <class T>
class SpatialWaypoint : public AbstractMissionItem
{
public:
    virtual MissionItemType getMissionType()const;

    virtual std::string getDescription()const;

    virtual bool hasSpatialMissionInfluence()const;

public:
    SpatialWaypoint();

public:
    T position;
};

} //end of namespace MissionItem
#endif // SPATIAL_WAYPOINT_H
