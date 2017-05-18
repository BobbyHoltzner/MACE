#include "spatial_loiter_turns.h"

namespace CommandItem {

template <class T>
Data::CommandItemType SpatialLoiter_Turns<T>::getCommandType() const
{
    return Data::CommandItemType::CI_NAV_LOITER_TURNS;
}

template <class T>
std::string SpatialLoiter_Turns<T>::getDescription() const
{
    return "This causes the vehicle to loiter around this MISSION for X turns";
}

template <class T>
bool SpatialLoiter_Turns<T>::hasSpatialInfluence() const
{
    return true;
}

//____________________________________________________________________________
template<>
SpatialLoiter_Turns<DataState::StateGlobalPosition>::SpatialLoiter_Turns()
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
}

template<>
SpatialLoiter_Turns<DataState::StateLocalPosition>::SpatialLoiter_Turns()
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_LOCAL_ENU;
}
//____________________________________________________________________________

//____________________________________________________________________________

template <class T>
SpatialLoiter_Turns<T>::SpatialLoiter_Turns(const SpatialLoiter_Turns<T> &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}
//____________________________________________________________________________

//____________________________________________________________________________
template<>
SpatialLoiter_Turns<DataState::StateGlobalPosition>::SpatialLoiter_Turns(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
}

template<>
SpatialLoiter_Turns<DataState::StateLocalPosition>::SpatialLoiter_Turns(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_LOCAL_ENU;
}
//____________________________________________________________________________

} //end of namespace CommandItem

template class CommandItem::SpatialLoiter_Turns<DataState::StateGlobalPosition>;
template class CommandItem::SpatialLoiter_Turns<DataState::StateLocalPosition>;
