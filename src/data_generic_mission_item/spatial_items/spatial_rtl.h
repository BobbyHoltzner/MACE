#ifndef SPATIAL_RTL_H
#define SPATIAL_RTL_H

#include "../abstract_mission_item.h"
#include "../mission_item_types.h"


namespace MissionItem {

class SpatialRTL : public AbstractMissionItem
{
public:
    virtual MissionItemType getMissionType();

    virtual std::string getDescription();

    virtual bool hasSpatialInfluence();

public:
    SpatialRTL();
};

} //end of namespace MissionItem

#endif // SPATIAL_RTL_H
