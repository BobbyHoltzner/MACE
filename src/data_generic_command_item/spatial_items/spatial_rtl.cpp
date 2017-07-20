#include "spatial_rtl.h"

namespace CommandItem {

Data::CommandItemType SpatialRTL::getCommandType() const
{
    return Data::CommandItemType::CI_NAV_RETURN_TO_LAUNCH;
}

std::string SpatialRTL::getDescription() const
{
    return "This causes the vehicle to return to the launch location";
}

bool SpatialRTL::hasSpatialInfluence() const
{
    return true;
}

SpatialRTL::SpatialRTL()
{

}

SpatialRTL::SpatialRTL(const SpatialRTL &obj):
    AbstractCommandItem(0,0)
{
    this->operator =(obj);
}

SpatialRTL::SpatialRTL(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget)
{

}


}
