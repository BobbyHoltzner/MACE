#include "spatial_loiter_time.h"

namespace MissionItem {

template <class T>
Data::MissionItemType SpatialLoiter_Time<T>::getMissionType() const
{
    return Data::MissionItemType::MI_NAV_LOITER_TIME;
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

template <>
std::ostream& SpatialLoiter_Time<DataState::StateGlobalPosition>::operator<<(std::ostream &out)
{
    out<<"Spatial Global Loiter Time(SystemID: "<<m_VehicleID<<", Radius: "<<radius<<", Time: "<<duration<<", Latitude: "<<position.latitude<<", Longitude: "<<position.longitude<<", Altitude: "<<position.altitude<<")";
    return out;
}

template <>
std::ostream& SpatialLoiter_Time<DataState::StateLocalPosition>::operator<<(std::ostream &out)
{
    out<<"Spatial Local Loiter Time(SystemID: "<<m_VehicleID<<", Radius: "<<radius<<", Time: "<<duration<<", Latitude: "<<position.x<<", Longitude: "<<position.y<<", Altitude: "<<position.z<<")";
    return out;
}

template<>
SpatialLoiter_Time<DataState::StateGlobalPosition>::SpatialLoiter_Time()
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
}

template<>
SpatialLoiter_Time<DataState::StateLocalPosition>::SpatialLoiter_Time()
{
    m_CoordinateFrame = CoordinateFrameType::CF_LOCAL_ENU;
}

} //end of namespace MissionItem

template class MissionItem::SpatialLoiter_Time<DataState::StateGlobalPosition>;
template class MissionItem::SpatialLoiter_Time<DataState::StateLocalPosition>;
