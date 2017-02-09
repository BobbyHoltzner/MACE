#include "spatial_rtl.h"

namespace MissionItem {

MissionItemType SpatialRTL::getMissionType() const
{
    return MissionItemType::RTL;
}

std::string SpatialRTL::getDescription() const
{
    return "This causes the vehicle to return to the launch location";
}

bool SpatialRTL::hasSpatialMissionInfluence() const
{
    return true;
}

}
