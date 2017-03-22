#include "spatial_land.h"

namespace MissionItem {

template <class T>
MissionItemType SpatialLand<T>::getMissionType() const
{
    return MissionItemType::LAND;
}

template <class T>
std::string SpatialLand<T>::getDescription() const
{
    return "This causes the vehicle to land either at the current location or prescribed location";
}

template <class T>
bool SpatialLand<T>::hasSpatialMissionInfluence() const
{
    return true;
}

template <>
std::ostream& SpatialLand<DataState::StateGlobalPosition>::operator<<(std::ostream &out)
{
    out<<"Spatial Global Land(SystemID: "<<m_VehicleID<<", Flag: "<<landFlag<<"Latitude: "<<position.latitude<<", Longitude: "<<position.longitude<<", Altitude: "<<position.altitude<<")";
    return out;
}

template <>
std::ostream& SpatialLand<DataState::StateLocalPosition>::operator<<(std::ostream &out)
{
    out<<"Spatial Local Land(SystemID: "<<m_VehicleID<<", Flag: "<<landFlag<<"Latitude: "<<position.x<<", Longitude: "<<position.y<<", Altitude: "<<position.z<<")";
    return out;
}

template<>
SpatialLand<DataState::StateGlobalPosition>::SpatialLand()
{
    m_PositionalFrame = Data::PositionalFrame::GLOBAL;
    m_CoordinateFrame = Data::CoordinateFrame::NED;
}

template<>
SpatialLand<DataState::StateLocalPosition>::SpatialLand()
{
    m_PositionalFrame = Data::PositionalFrame::LOCAL;
    m_CoordinateFrame = Data::CoordinateFrame::NED;
}

template <class T>
bool SpatialLand<T>::getLandFlag() const
{
    return landFlag;
}

template <class T>
void SpatialLand<T>::setLandFlag(const bool &landFlag)
{
    this->landFlag = landFlag;
}

} //end of namespace MissionItem

template class MissionItem::SpatialLand<DataState::StateGlobalPosition>;
template class MissionItem::SpatialLand<DataState::StateLocalPosition>;
