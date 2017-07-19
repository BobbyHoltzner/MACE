#include "spatial_land.h"

namespace CommandItem {

Data::CommandItemType SpatialLand::getCommandType() const
{
    return Data::CommandItemType::CI_NAV_LAND;
}

std::string SpatialLand::getDescription() const
{
    return "This causes the vehicle to land either at the current location or prescribed location";
}

bool SpatialLand::hasSpatialInfluence() const
{
    return true;
}

SpatialLand::SpatialLand():
    AbstractCommandItem(0,0)
{

}

SpatialLand::SpatialLand(const SpatialLand &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

SpatialLand::SpatialLand(const int &systemOrigin,  const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}


} //end of namespace CommandItem
