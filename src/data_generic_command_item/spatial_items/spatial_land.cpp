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

std::ostream& operator<<(std::ostream& os, const SpatialLand& t)
{
    std::stringstream stream;
    stream.precision(6);
    stream << std::fixed << "Spatial Land: " << t.position.getX() << ", "<< t.position.getY() << ", "<< t.position.getZ() << ".";
    os << stream.str();

    return os;
}

} //end of namespace CommandItem
