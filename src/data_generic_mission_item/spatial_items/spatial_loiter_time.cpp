#include "spatial_loiter_time.h"

namespace MissionItem {

template <class T>
MissionItemType SpatialLoiter_Time<T>::getMissionType() const
{
    return MissionItemType::LOITER_TIME;
}

template <class T>
std::string SpatialLoiter_Time<T>::getDescription() const
{
    return "This causes the vehicle to loiter around this MISSION for X seconds";
}

template <class T>
bool SpatialLoiter_Time<T>::hasSpatialMissionInfluence() const
{
    return true;
}

template<>
SpatialLoiter_Time<DataState::StateGlobalPosition>::SpatialLoiter_Time()
{
    m_PositionalFrame = Data::PositionalFrame::GLOBAL;
    m_CoordinateFrame = Data::CoordinateFrame::NED;
}

template<>
SpatialLoiter_Time<DataState::StateLocalPosition>::SpatialLoiter_Time()
{
    m_PositionalFrame = Data::PositionalFrame::LOCAL;
    m_CoordinateFrame = Data::CoordinateFrame::NED;
}

} //end of namespace MissionItem

template class MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition>;
template class MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition>;
