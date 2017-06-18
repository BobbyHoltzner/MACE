#include "spatial_waypoint.h"

namespace CommandItem {
template <class T>
Data::CommandItemType SpatialWaypoint<T>::getCommandType() const
{
    return Data::CommandItemType::CI_NAV_WAYPOINT;
}

template <class T>
std::string SpatialWaypoint<T>::getDescription() const
{
    return "This is a waypoint mission item for a vehicle";
}

template <class T>
bool SpatialWaypoint<T>::hasSpatialInfluence() const
{
    return true;
}

template<class T>
SpatialWaypoint<T>::SpatialWaypoint():
    AbstractCommandItem(0,0), DataState::StateGenericPosition<T>()
{

}

template<class T>
SpatialWaypoint<T>::SpatialWaypoint(const SpatialWaypoint<T> &obj):
    AbstractCommandItem(0,0), DataState::StateGenericPosition<T>()
{
    this->operator =(obj);
}

template<class T>
SpatialWaypoint<T>::SpatialWaypoint(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget), DataState::StateGenericPosition<T>()
{

}

} //end of namepsace CommandItem
