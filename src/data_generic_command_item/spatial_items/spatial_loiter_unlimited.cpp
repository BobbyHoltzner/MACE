#include "spatial_loiter_unlimited.h"

namespace CommandItem {

template <class T>
Data::CommandItemType SpatialLoiter_Unlimited<T>::getCommandType() const
{
    return Data::CommandItemType::CI_NAV_LOITER_UNLIM;
}

template <class T>
std::string SpatialLoiter_Unlimited<T>::getDescription() const
{
    return "This causes the vehicle to loiter around this MISSION an unlimited amount of time";
}

template <class T>
bool SpatialLoiter_Unlimited<T>::hasSpatialInfluence() const
{
    return true;
}

//____________________________________________________________________________
template<>
SpatialLoiter_Unlimited<DataState::StateGlobalPosition>::SpatialLoiter_Unlimited()
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
}

template<>
SpatialLoiter_Unlimited<DataState::StateLocalPosition>::SpatialLoiter_Unlimited()
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_LOCAL_ENU;
}
//____________________________________________________________________________

//____________________________________________________________________________
template<class T>
SpatialLoiter_Unlimited<T>::SpatialLoiter_Unlimited(const SpatialLoiter_Unlimited<T> &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}
//____________________________________________________________________________

//____________________________________________________________________________
template<>
SpatialLoiter_Unlimited<DataState::StateGlobalPosition>::SpatialLoiter_Unlimited(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
}

template<>
SpatialLoiter_Unlimited<DataState::StateLocalPosition>::SpatialLoiter_Unlimited(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_LOCAL_ENU;
}
//____________________________________________________________________________


} //end of namespace CommandItem

template class CommandItem::SpatialLoiter_Unlimited<DataState::StateGlobalPosition>;
template class CommandItem::SpatialLoiter_Unlimited<DataState::StateLocalPosition>;
