#include "spatial_land.h"

namespace MissionItem {

MissionItemType SpatialLand::getMissionType()
{
    return MissionItemType::LAND;
}

std::string SpatialLand::getDescription()
{
    return "This causes the vehicle to land either at the current location or prescribed location";
}

bool SpatialLand::hasSpatialInfluence()
{
    return true;
}

}

