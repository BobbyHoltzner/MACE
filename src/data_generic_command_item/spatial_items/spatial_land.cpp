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

template<class T>
SpatialLand<T>::SpatialLand():
    AbstractCommandItem(0,0),DataState::StateGenericPosition<T>()
{

}

template<class T>
SpatialLand<T>::SpatialLand(const SpatialLand<T> &obj):
    AbstractCommandItem(0,0),DataState::StateGenericPosition<T>()
{
    this->operator =(obj);
}

template<class T>
SpatialLand<T>::SpatialLand(const int &systemOrigin,  const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget),DataState::StateGenericPosition<T>()
{

}


} //end of namespace CommandItem
