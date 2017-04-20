#include "spatial_takeoff.h"

namespace MissionItem {
template <class T>
SpatialTakeoff<T>::SpatialTakeoff() :
    positionFlag(true)
{

}

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

template <>
std::ostream& SpatialTakeoff<DataState::StateGlobalPosition>::operator<<(std::ostream &out)
{
    out<<"Spatial Global Takeoff(SystemID: "<<m_VehicleID<<", Latitude: "<<position.latitude<<", Longitude: "<<position.longitude<<", Altitude: "<<position.altitude<<")";
    return out;
}

template <>
std::ostream& SpatialTakeoff<DataState::StateLocalPosition>::operator<<(std::ostream &out)
{
    out<<"Spatial Local Takeoff(SystemID: "<<m_VehicleID<<", Latitude: "<<position.x<<", Longitude: "<<position.y<<", Altitude: "<<position.z<<")";
    return out;
}

}

template class MissionItem::SpatialTakeoff<DataState::StateGlobalPosition>;
template class MissionItem::SpatialTakeoff<DataState::StateLocalPosition>;
