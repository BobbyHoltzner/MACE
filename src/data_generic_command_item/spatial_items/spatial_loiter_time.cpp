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


//____________________________________________________________________________
template<>
SpatialLoiter_Time<DataState::StateGlobalPosition>::SpatialLoiter_Time()
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
}

template<>
SpatialLoiter_Time<DataState::StateLocalPosition>::SpatialLoiter_Time()
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_LOCAL_ENU;
}
//____________________________________________________________________________

//____________________________________________________________________________
template <class T>
SpatialLoiter_Time<T>::SpatialLoiter_Time(const SpatialLoiter_Time<T> &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}
//____________________________________________________________________________

//____________________________________________________________________________
template<>
SpatialLoiter_Time<DataState::StateGlobalPosition>::SpatialLoiter_Time(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
}

template<>
SpatialLoiter_Time<DataState::StateLocalPosition>::SpatialLoiter_Time(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_LOCAL_ENU;
}
//____________________________________________________________________________


} //end of namespace CommandItem

template class CommandItem::SpatialLoiter_Time<DataState::StateGlobalPosition>;
template class CommandItem::SpatialLoiter_Time<DataState::StateLocalPosition>;
