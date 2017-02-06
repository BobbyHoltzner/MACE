#include "spatial_waypoint.h"

namespace MissionItem {

template<>
SpatialWaypoint<DataState::StateGlobalPosition>::SpatialWaypoint()
{
    m_PositionalFrame = Data::PositionalFrame::GLOBAL;
}

SpatialWaypoint<DataState::StateLocalPosition>::SpatialWaypoint()
{
    m_PositionalFrame = Data::PositionalFrame::LOCAL;
}

} //end of namepsace MissionItem

template class MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>;
template class MissionItem::SpatialWaypoint<DataState::StateLocalPosition>;
