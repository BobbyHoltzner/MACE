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
    AbstractCommandItem(0,0)
{

}

SpatialHome::SpatialHome(const SpatialHome &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

SpatialHome::SpatialHome(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

std::ostream& operator<<(std::ostream& os, const SpatialHome& t)
{
    os << 'Spatial Home: ' << t.position.getX() << ', '<< t.position.getY() << ', '<< t.position.getZ() << '.';
    return os;
}

} //end of namespace CommandItem
