#include "spatial_home.h"

namespace CommandItem {

template<class T>
Data::CommandItemType SpatialHome<T>::getCommandType() const
{
    return Data::CommandItemType::CI_NAV_HOME;
}

template<class T>
std::string SpatialHome<T>::getDescription() const
{
    return "This stores the home location for a vehicle";
}

template<class T>
bool SpatialHome<T>::hasSpatialInfluence() const
{
    return true;
}

template<class T>
SpatialHome<T>::SpatialHome():
    AbstractCommandItem(0,0),DataState::StateGenericPosition<T>()
{

}

template<class T>
SpatialHome<T>::SpatialHome(const SpatialHome &obj):
    AbstractCommandItem(0,0),DataState::StateGenericPosition<T>()
{
    this->operator =(obj);
}

template<class T>
SpatialHome<T>::SpatialHome(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget),DataState::StateGenericPosition<T>()
{

}

} //end of namespace CommandItem
