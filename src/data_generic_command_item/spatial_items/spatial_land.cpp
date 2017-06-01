#include "spatial_land.h"

namespace CommandItem {

template <class T>
Data::CommandItemType SpatialLand<T>::getCommandType() const
{
    return Data::CommandItemType::CI_NAV_LAND;
}

template <class T>
std::string SpatialLand<T>::getDescription() const
{
    return "This causes the vehicle to land either at the current location or prescribed location";
}

template <class T>
bool SpatialLand<T>::hasSpatialInfluence() const
{
    return true;
}

//____________________________________________________________________________
template<>
SpatialLand<DataState::StateGlobalPosition>::SpatialLand()
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
}

template<>
SpatialLand<DataState::StateLocalPosition>::SpatialLand()
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_LOCAL_ENU;
}
//____________________________________________________________________________

//____________________________________________________________________________
template<>
SpatialLand<DataState::StateGlobalPosition>::SpatialLand(const SpatialLand<DataState::StateGlobalPosition> &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

template<>
SpatialLand<DataState::StateLocalPosition>::SpatialLand(const SpatialLand<DataState::StateLocalPosition> &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}
//____________________________________________________________________________

//____________________________________________________________________________
template<>
SpatialLand<DataState::StateGlobalPosition>::SpatialLand(const int &systemOrigin,  const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
}

template<>
SpatialLand<DataState::StateLocalPosition>::SpatialLand(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_LOCAL_ENU;
}
//____________________________________________________________________________

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

} //end of namespace CommandItem

template class CommandItem::SpatialLand<DataState::StateGlobalPosition>;
template class CommandItem::SpatialLand<DataState::StateLocalPosition>;
