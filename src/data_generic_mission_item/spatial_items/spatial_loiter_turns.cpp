#include "spatial_loiter_turns.h"

namespace MissionItem {

template <class T>
Data::MissionItemType SpatialLoiter_Turns<T>::getMissionType() const
{
    return Data::MissionItemType::MI_NAV_LOITER_TURNS;
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

template <>
std::ostream& SpatialLoiter_Turns<DataState::StateGlobalPosition>::operator<<(std::ostream &out)
{
    out<<"Spatial Global Loiter Turns(SystemID: "<<m_VehicleID<<", Radius: "<<radius<<", Turns: "<<turns<<", Latitude: "<<position.latitude<<", Longitude: "<<position.longitude<<", Altitude: "<<position.altitude<<")";
    return out;
}

template <>
std::ostream& SpatialLoiter_Turns<DataState::StateLocalPosition>::operator<<(std::ostream &out)
{
    out<<"Spatial Local Loiter Turns(SystemID: "<<m_VehicleID<<", Radius: "<<radius<<", Turns: "<<turns<<", Latitude: "<<position.x<<", Longitude: "<<position.y<<", Altitude: "<<position.z<<")";
    return out;
}

template<>
SpatialLoiter_Turns<DataState::StateGlobalPosition>::SpatialLoiter_Turns()
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
}

template<>
SpatialLoiter_Turns<DataState::StateLocalPosition>::SpatialLoiter_Turns()
{
    m_CoordinateFrame = CoordinateFrameType::CF_LOCAL_ENU;
}

} //end of namespace MissionItem

template class MissionItem::SpatialLoiter_Turns<DataState::StateGlobalPosition>;
template class MissionItem::SpatialLoiter_Turns<DataState::StateLocalPosition>;
