#include "spatial_rtl.h"

namespace MissionItem {

MissionItemType SpatialRTL::getMissionType()
{
    return MissionItemType::RTL;
}

std::string SpatialRTL::getDescription()
{
    return "This causes the vehicle to return to the launch location";
}

bool SpatialRTL::hasSpatialInfluence()
{
    return true;
}

}
