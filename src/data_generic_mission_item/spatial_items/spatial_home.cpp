#include "spatial_home.h"

namespace MissionItem {

MissionItemType SpatialHome::getMissionType() const
{
    return MissionItemType::RTL;
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

}

SpatialHome::SpatialHome(const SpatialHome &spatialHome)
{
    this->position = spatialHome.position;
}
}
