#include "spatial_loiter_time.h"

namespace CommandItem {

template <class T>
Data::CommandItemType SpatialLoiter_Time<T>::getCommandType() const
{
    return Data::CommandItemType::CI_NAV_LOITER_TIME;
}

template <class T>
std::string SpatialLoiter_Time<T>::getDescription() const
{
    return "This causes the vehicle to loiter around this MISSION for X seconds";
}

template <class T>
bool SpatialLoiter_Time<T>::hasSpatialInfluence() const
{
    return true;
}


template <class T>
SpatialLoiter_Time<T>::SpatialLoiter_Time():
    AbstractCommandItem(0,0),DataState::StateGenericPosition<T>()
{

}

template <class T>
SpatialLoiter_Time<T>::SpatialLoiter_Time(const SpatialLoiter_Time<T> &obj):
    AbstractCommandItem(0,0), DataState::StateGenericPosition<T>()
{
    this->operator =(obj);
}

template<class T>
SpatialLoiter_Time<T>::SpatialLoiter_Time(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget), DataState::StateGenericPosition<T>()
{

}


} //end of namespace CommandItem
