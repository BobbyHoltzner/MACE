#include "spatial_waypoint.h"

namespace CommandItem {

Data::CommandItemType SpatialWaypoint::getCommandType() const
{
    return Data::CommandItemType::CI_NAV_WAYPOINT;
}

std::string SpatialWaypoint::getDescription() const
{
    return "This is a waypoint mission item for a vehicle";
}

bool SpatialWaypoint::hasSpatialInfluence() const
{
    return true;
}

SpatialWaypoint::SpatialWaypoint():
    AbstractCommandItem(0,0)
{

}

SpatialWaypoint::SpatialWaypoint(const SpatialWaypoint &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

SpatialWaypoint::SpatialWaypoint(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}

} //end of namepsace CommandItem
