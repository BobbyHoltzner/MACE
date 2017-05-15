#include "spatial_home.h"

namespace MissionItem {

MissionItemType SpatialHome::getMissionType() const
{
    return MissionItemType::MI_NAV_HOME;
}

std::string SpatialHome::getDescription() const
{
    return "This stores the home location for a vehicle";
}

bool SpatialHome::hasSpatialMissionInfluence() const
{
    return true;
}

SpatialHome::SpatialHome()
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
}

SpatialHome::SpatialHome(const SpatialHome &spatialHome)
{
    this->operator =(spatialHome);
}
}
