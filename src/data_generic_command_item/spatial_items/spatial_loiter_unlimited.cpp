#include "spatial_loiter_unlimited.h"

namespace CommandItem {

COMMANDITEM SpatialLoiter_Unlimited::getCommandType() const
{
    return COMMANDITEM::CI_NAV_LOITER_UNLIM;
}

std::string SpatialLoiter_Unlimited::getDescription() const
{
    return "This causes the vehicle to loiter around this MISSION an unlimited amount of time";
}

bool SpatialLoiter_Unlimited::hasSpatialInfluence() const
{
    return true;
}

AbstractCommandItem* SpatialLoiter_Unlimited::getClone() const
{
    return (new SpatialLoiter_Unlimited(*this));
}

void SpatialLoiter_Unlimited::getClone(AbstractCommandItem** command) const
{
    *command = new SpatialLoiter_Unlimited(*this);
}

SpatialLoiter_Unlimited::SpatialLoiter_Unlimited():
    AbstractCommandItem(0,0)
{

}

SpatialLoiter_Unlimited::SpatialLoiter_Unlimited(const SpatialLoiter_Unlimited &obj):
    AbstractCommandItem(obj), AbstractSpatialPosition(obj)
{
    this->operator =(obj);
}

SpatialLoiter_Unlimited::SpatialLoiter_Unlimited(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget), AbstractSpatialPosition()
{

}


} //end of namespace CommandItem

