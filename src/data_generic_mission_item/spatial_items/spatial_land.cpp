#include "spatial_land.h"

namespace MissionItem {

MissionItemType SpatialLand::getMissionType() const
{
    return MissionItemType::LAND;
}

std::string SpatialLand::getDescription() const
{
    return "This causes the vehicle to land either at the current location or prescribed location";
}

bool SpatialLand::hasSpatialMissionInfluence() const
{
    return true;
}

bool SpatialLand::getLandFlag()
{
    return landFlag;
}

void SpatialLand::setLandFlag(const bool &landFlag)
{
    this->landFlag = landFlag;
}

}

template class MissionItem::SpatialLand<DataState::StateGlobalPosition>;
template class MissionItem::SpatialLand<DataState::StateLocalPosition>;
