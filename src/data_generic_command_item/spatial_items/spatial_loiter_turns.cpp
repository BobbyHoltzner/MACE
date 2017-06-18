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

template<class T>
SpatialLoiter_Turns<T>::SpatialLoiter_Turns():
    AbstractCommandItem(0,0), DataState::StateGenericPosition<T>()
{

}


template <class T>
SpatialLoiter_Turns<T>::SpatialLoiter_Turns(const SpatialLoiter_Turns<T> &obj):
    AbstractCommandItem(0,0), DataState::StateGenericPosition<T>()
{
    this->operator =(obj);
}

template<class T>
SpatialLoiter_Turns<T>::SpatialLoiter_Turns(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget), DataState::StateGenericPosition<T>()
{

}

} //end of namespace CommandItem

