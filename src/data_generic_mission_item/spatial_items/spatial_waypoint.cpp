#include "spatial_waypoint.h"

namespace MissionItem {

template<>
SpatialWaypoint<DataState::StateGlobalPosition>::SpatialWaypoint()
{
    m_PositionalFrame = Data::PositionalFrame::GLOBAL;
    m_CoordinateFrame = Data::CoordinateFrame::NED;
}

template<>
SpatialWaypoint<DataState::StateLocalPosition>::SpatialWaypoint()
{
    m_PositionalFrame = Data::PositionalFrame::LOCAL;
    m_CoordinateFrame = Data::CoordinateFrame::NED;
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

template <>
std::ostream& SpatialWaypoint<DataState::StateGlobalPosition>::operator<<(std::ostream &out)
{
    out<<"Spatial Global Waypoint(SystemID: "<<m_VehicleID<<", Latitude: "<<position.latitude<<", Longitude: "<<position.longitude<<", Altitude: "<<position.altitude<<")";
    return out;
}

template <>
std::ostream& SpatialWaypoint<DataState::StateLocalPosition>::operator<<(std::ostream &out)
{
    out<<"Spatial Local Waypoint(SystemID: "<<m_VehicleID<<", Latitude: "<<position.x<<", Longitude: "<<position.y<<", Altitude: "<<position.z<<")";
    return out;
}

} //end of namepsace MissionItem

//template class MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>;
//template class MissionItem::SpatialWaypoint<DataState::StateLocalPosition>;
