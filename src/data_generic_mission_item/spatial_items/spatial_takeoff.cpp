#include "spatial_takeoff.h"

namespace MissionItem {

MissionItemType SpatialTakeoff::getMissionType() const
{
    return MissionItemType::TAKEOFF;
}

std::string SpatialTakeoff::getDescription() const
{
    return "This causes the vehicle to perform a takeoff action";
}

bool SpatialTakeoff::hasSpatialMissionInfluence() const
{
    return true;
}

}

template class MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>;
template class MissionItem::SpatialTakeoff<DataState::StateLocalPosition>;
