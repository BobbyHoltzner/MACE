#include "spatial_waypoint.h"

namespace CommandItem {

COMMANDITEM SpatialWaypoint::getCommandType() const
{
    return COMMANDITEM::CI_NAV_WAYPOINT;
}

std::string SpatialWaypoint::getDescription() const
{
    return "This is a waypoint mission item for a vehicle";
}

bool SpatialWaypoint::hasSpatialInfluence() const
{
    return true;
}

std::shared_ptr<AbstractCommandItem> SpatialWaypoint::getClone() const
{
    return std::make_shared<SpatialWaypoint>(*this);
}

void SpatialWaypoint::getClone(std::shared_ptr<AbstractCommandItem> &command) const
{
    command = std::make_shared<SpatialWaypoint>(*this);
}

SpatialWaypoint::SpatialWaypoint():
    AbstractCommandItem(0,0), AbstractSpatialPosition()
{

}

SpatialWaypoint::SpatialWaypoint(const SpatialWaypoint &obj):
    AbstractCommandItem(0,0), AbstractSpatialPosition(obj)
{
    this->operator =(obj);
}

SpatialWaypoint::SpatialWaypoint(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget), AbstractSpatialPosition()
{

}

SpatialWaypoint::~SpatialWaypoint()
{

}

std::ostream& operator<<(std::ostream& os, const SpatialWaypoint& t)
{
    std::stringstream stream;
    stream.precision(6);
    stream << std::fixed << "Spatial Waypoint: " << t.position->getX() << ", "<< t.position->getY() << ", "<< t.position->getZ() << ".";
    os << stream.str();

    return os;
}

} //end of namepsace CommandItem
