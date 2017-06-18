#include "spatial_takeoff.h"

namespace CommandItem {

template <class T>
Data::CommandItemType SpatialTakeoff<T>::getCommandType() const
{
    return Data::CommandItemType::CI_NAV_TAKEOFF;
}

template <class T>
std::string SpatialTakeoff<T>::getDescription() const
{
    return "This causes the vehicle to perform a takeoff action";
}

template <class T>
bool SpatialTakeoff<T>::hasSpatialInfluence() const
{
    return true;
}

//____________________________________________________________________________
template<class T>
SpatialTakeoff<T>::SpatialTakeoff():
    AbstractCommandItem(0,0), DataState::StateGenericPosition<T>()
{

}

template<class T>
SpatialTakeoff<T>::SpatialTakeoff(const SpatialTakeoff<T> &obj):
    AbstractCommandItem(0,0), DataState::StateGenericPosition<T>()
{
    this->operator =(obj);
}

template<class T>
SpatialTakeoff<T>::SpatialTakeoff(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget), DataState::StateGenericPosition<T>()
{

}

}
