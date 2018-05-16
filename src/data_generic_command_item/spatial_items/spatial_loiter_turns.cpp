#include "spatial_loiter_turns.h"

namespace CommandItem {

COMMANDITEM SpatialLoiter_Turns::getCommandType() const
{
    return COMMANDITEM::CI_NAV_LOITER_TURNS;
}

std::string SpatialLoiter_Turns::getDescription() const
{
    return "This causes the vehicle to loiter around this MISSION for X turns";
}

bool SpatialLoiter_Turns::hasSpatialInfluence() const
{
    return true;
}

AbstractCommandItem* SpatialLoiter_Turns::getClone() const
{
    return (new SpatialLoiter_Turns(*this));
}

void SpatialLoiter_Turns::getClone(AbstractCommandItem** command) const
{
    *command = new SpatialLoiter_Turns(*this);
}

SpatialLoiter_Turns::SpatialLoiter_Turns():
    AbstractCommandItem(0,0), AbstractSpatialPosition()
{

}


SpatialLoiter_Turns::SpatialLoiter_Turns(const SpatialLoiter_Turns &obj):
    AbstractCommandItem(obj), AbstractSpatialPosition(obj)
{
    this->operator =(obj);
}

SpatialLoiter_Turns::SpatialLoiter_Turns(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget), AbstractSpatialPosition()
{

}

} //end of namespace CommandItem

