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

SpatialHome::SpatialHome()
{
    m_CoordinateFrame = Data::CoordinateFrameType::CF_GLOBAL_RELATIVE_ALT;
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

} //end of namespace CommandItem
