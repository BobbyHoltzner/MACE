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
void SpatialWaypoint<DataState::StateGlobalPosition>::print()
{
    std::cout<<"Spatial Global Waypoint(SystemID: "<<m_VehicleID<<", Latitude: "<<position.latitude<<", Longitude: "<<position.longitude<<", Altitude: "<<position.altitude<<")"<<std::endl;
}

template <>
void SpatialWaypoint<DataState::StateLocalPosition>::print()
{
    std::cout<<"Spatial Local Waypoint(SystemID: "<<m_VehicleID<<", Latitude: "<<position.x<<", Longitude: "<<position.y<<", Altitude: "<<position.z<<")"<<std::endl;
}

} //end of namepsace MissionItem

//template class MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>;
//template class MissionItem::SpatialWaypoint<DataState::StateLocalPosition>;
