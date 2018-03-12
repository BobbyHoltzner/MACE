#include "spatial_rtl.h"

namespace CommandItem {

COMMANDITEM SpatialRTL::getCommandType() const
{
    return COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH;
}

std::string SpatialRTL::getDescription() const
{
    return "This causes the vehicle to return to the launch location";
}

bool SpatialRTL::hasSpatialInfluence() const
{
    return true;
}

AbstractCommandItem* SpatialRTL::getClone() const
{
    return (new SpatialRTL(*this));
}

void SpatialRTL::getClone(AbstractCommandItem** command) const
{
    *command = new SpatialRTL(*this);
}

SpatialRTL::SpatialRTL():
    AbstractCommandItem(0,0)
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
