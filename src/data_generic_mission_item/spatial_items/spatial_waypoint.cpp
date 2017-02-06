#include "spatial_waypoint.h"

namespace MissionItem {

template<>
SpatialWaypoint<DataState::StateGlobalPosition>::SpatialWaypoint()
{
    m_PositionalFrame = Data::PositionalFrame::GLOBAL;
}

template<>
SpatialWaypoint<DataState::StateLocalPosition>::SpatialWaypoint()
{
    m_PositionalFrame = Data::PositionalFrame::LOCAL;
}

template <class T>
MissionItemType SpatialWaypoint<T>::getMissionType() const
{
    return MissionItemType::WAYPOINT;
}

template <class T>
std::string SpatialWaypoint<T>::getDescription() const
{
    return "This is a waypoint mission item for a vehicle";
}

template <class T>
bool SpatialWaypoint<T>::hasSpatialMissionInfluence() const
{
    return true;
}

} //end of namepsace MissionItem

//template class MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>;
//template class MissionItem::SpatialWaypoint<DataState::StateLocalPosition>;
