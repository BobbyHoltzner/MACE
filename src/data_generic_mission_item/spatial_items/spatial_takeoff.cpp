#include "spatial_takeoff.h"

namespace MissionItem {

MissionItemType SpatialTakeoff::getMissionType()
{
    return MissionItemType::TAKEOFF;
}

std::string SpatialTakeoff::getDescription()
{
    return "This causes the vehicle to perform a takeoff action";
}

bool SpatialTakeoff::hasSpatialInfluence()
{
    return true;
}

}

