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

//____________________________________________________________________________
template<>
SpatialWaypoint<DataState::StateGlobalPosition>::SpatialWaypoint()
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
}

template<>
SpatialWaypoint<DataState::StateLocalPosition>::SpatialWaypoint()
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_LOCAL_ENU;
}
//____________________________________________________________________________

//____________________________________________________________________________
template<class T>
SpatialWaypoint<T>::SpatialWaypoint(const SpatialWaypoint<T> &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}
//____________________________________________________________________________

//____________________________________________________________________________
template<>
SpatialWaypoint<DataState::StateGlobalPosition>::SpatialWaypoint(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
}

template<>
SpatialWaypoint<DataState::StateLocalPosition>::SpatialWaypoint(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_LOCAL_ENU;
}
//____________________________________________________________________________


} //end of namepsace CommandItem

template class CommandItem::SpatialWaypoint<DataState::StateGlobalPosition>;
template class CommandItem::SpatialWaypoint<DataState::StateLocalPosition>;
