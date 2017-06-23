#include "spatial_home.h"

namespace CommandItem {

Data::CommandItemType SpatialHome::getCommandType() const
{
    return Data::CommandItemType::CI_NAV_HOME;
}

std::string SpatialHome::getDescription() const
{
    return "This stores the home location for a vehicle";
}

bool SpatialHome::hasSpatialInfluence() const
{
    return true;
}

SpatialHome::SpatialHome():
    AbstractCommandItem(0,0),DataState::StateGenericPosition()
{

}

SpatialHome::SpatialHome(const SpatialHome &obj):
    AbstractCommandItem(0,0),DataState::StateGenericPosition()
{
    this->operator =(obj);
}

SpatialHome::SpatialHome(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget),DataState::StateGenericPosition()
{

}

} //end of namespace CommandItem
