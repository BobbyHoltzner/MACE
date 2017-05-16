#include "spatial_rtl.h"

namespace MissionItem {

Data::MissionItemType SpatialRTL::getMissionType() const
{
    return Data::MissionItemType::MI_NAV_RETURN_TO_LAUNCH;
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
