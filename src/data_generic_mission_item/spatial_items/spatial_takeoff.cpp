#include "spatial_takeoff.h"

namespace MissionItem {
template <class T>
MissionItemType SpatialTakeoff<T>::getMissionType() const
{
    return MissionItemType::TAKEOFF;
}

template <class T>
std::string SpatialTakeoff<T>::getDescription() const
{
    return "This causes the vehicle to perform a takeoff action";
}

template <class T>
bool SpatialTakeoff<T>::hasSpatialMissionInfluence() const
{
    return true;
}

}

template class MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>;
template class MissionItem::SpatialTakeoff<DataState::StateLocalPosition>;
