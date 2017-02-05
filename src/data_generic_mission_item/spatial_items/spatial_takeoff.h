#ifndef SPATIAL_TAKEOFF_H
#define SPATIAL_TAKEOFF_H

#include "../abstract_mission_item.h"
#include "../mission_item_types.h"

#include "data_generic_state_item/state_global_position.h"
#include "data_generic_state_item/state_local_position.h"

namespace MissionItem {

class SpatialTakeoff
{
public:
    virtual MissionItemType getMissionType();

    virtual std::string getDescription();

    virtual bool hasSpatialInfluence();

public:
    SpatialTakeoff();
};

} //end of namespace MissionItem

#endif // SPATIAL_TAKEOFF_H
