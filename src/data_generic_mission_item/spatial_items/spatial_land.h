#ifndef SPATIAL_LAND_H
#define SPATIAL_LAND_H

#include "../abstract_mission_item.h"
#include "../mission_item_types.h"

#include "data_generic_state_item/state_global_position.h"
#include "data_generic_state_item/state_local_position.h"

namespace MissionItem {

class SpatialLand : public AbstractMissionItem
{
public:
    virtual MissionItemType getMissionType();

    virtual std::string getDescription();

    virtual bool hasSpatialInfluence();

public:
    SpatialLand();
};

} //end of namespace MissionItem

#endif // SPATIAL_LAND_H
