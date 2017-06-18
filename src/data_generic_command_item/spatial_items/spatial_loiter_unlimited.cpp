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

template<class T>
SpatialLoiter_Unlimited<T>::SpatialLoiter_Unlimited():
    AbstractCommandItem(0,0), DataState::StateGenericPosition<T>()
{

}

template<class T>
SpatialLoiter_Unlimited<T>::SpatialLoiter_Unlimited(const SpatialLoiter_Unlimited<T> &obj):
    AbstractCommandItem(0,0), DataState::StateGenericPosition<T>()
{
    this->operator =(obj);
}

template<class T>
SpatialLoiter_Unlimited<T>::SpatialLoiter_Unlimited(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget), DataState::StateGenericPosition<T>()
{

}


} //end of namespace CommandItem

