#include "spatial_loiter_turns.h"

namespace MissionItem {

template <class T>
MissionItemType SpatialLoiter_Turns<T>::getMissionType() const
{
    return MissionItemType::LOITER_TURNS;
}

template <class T>
std::string SpatialLoiter_Turns<T>::getDescription() const
{
    return "This causes the vehicle to loiter around this MISSION for X turns";
}

template <class T>
bool SpatialLoiter_Turns<T>::hasSpatialMissionInfluence() const
{
    return true;
}

template<>
SpatialLoiter_Turns<DataState::StateGlobalPosition>::SpatialLoiter_Turns()
{
    m_PositionalFrame = Data::PositionalFrame::GLOBAL;
    m_CoordinateFrame = Data::CoordinateFrame::NED;
}

template<>
SpatialLoiter_Turns<DataState::StateLocalPosition>::SpatialLoiter_Turns()
{
    m_PositionalFrame = Data::PositionalFrame::LOCAL;
    m_CoordinateFrame = Data::CoordinateFrame::NED;
}

} //end of namespace MissionItem

template class MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition>;
template class MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition>;
