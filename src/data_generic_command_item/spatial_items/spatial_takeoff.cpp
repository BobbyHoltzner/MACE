#include "spatial_takeoff.h"

namespace CommandItem {

Data::CommandItemType SpatialTakeoff::getCommandType() const
{
    return Data::CommandItemType::CI_NAV_TAKEOFF;
}

std::string SpatialTakeoff::getDescription() const
{
    return "This causes the vehicle to perform a takeoff action";
}

bool SpatialTakeoff::hasSpatialInfluence() const
{
    return true;
}

SpatialTakeoff::SpatialTakeoff():
    AbstractCommandItem(0,0)
{

}

SpatialTakeoff::SpatialTakeoff(const SpatialTakeoff &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

SpatialTakeoff::SpatialTakeoff(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

std::ostream& operator<<(std::ostream& os, const SpatialTakeoff& t)
{
    os << 'Spatial Home: ' << t.position.getX() << ', '<< t.position.getY() << ', '<< t.position.getZ() << '.';
    return os;
}

}
