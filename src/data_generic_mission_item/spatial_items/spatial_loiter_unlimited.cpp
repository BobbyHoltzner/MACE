#include "spatial_loiter_unlimited.h"

namespace MissionItem {

template <class T>
MissionItemType SpatialLoiter_Unlimited<T>::getMissionType() const
{
    return MissionItemType::LOITER_UNLIMITED;
}

template <class T>
std::string SpatialLoiter_Unlimited<T>::getDescription() const
{
    return "This causes the vehicle to loiter around this MISSION an unlimited amount of time";
}

template <class T>
bool SpatialLoiter_Unlimited<T>::hasSpatialMissionInfluence() const
{
    return true;
}

template <>
std::ostream& SpatialLoiter_Unlimited<DataState::StateGlobalPosition>::operator<<(std::ostream &out)
{
    out<<"Spatial Global Loiter Unlimited(SystemID: "<<m_VehicleID<<", Radius: "<<radius<<", Latitude: "<<position.latitude<<", Longitude: "<<position.longitude<<", Altitude: "<<position.altitude<<")";
    return out;
}

template <>
std::ostream& SpatialLoiter_Unlimited<DataState::StateLocalPosition>::operator<<(std::ostream &out)
{
    out<<"Spatial Local Loiter Unlimited(SystemID: "<<m_VehicleID<<", Radius: "<<radius<<", Latitude: "<<position.x<<", Longitude: "<<position.y<<", Altitude: "<<position.z<<")";
    return out;
}

template<>
SpatialLoiter_Unlimited<DataState::StateGlobalPosition>::SpatialLoiter_Unlimited()
{
    m_PositionalFrame = Data::PositionalFrame::GLOBAL;
    m_CoordinateFrame = Data::CoordinateFrame::NED;
}

template<>
SpatialLoiter_Unlimited<DataState::StateLocalPosition>::SpatialLoiter_Unlimited()
{
    m_PositionalFrame = Data::PositionalFrame::LOCAL;
    m_CoordinateFrame = Data::CoordinateFrame::NED;
}

} //end of namespace MissionItem

template class MissionItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition>;
template class MissionItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition>;
