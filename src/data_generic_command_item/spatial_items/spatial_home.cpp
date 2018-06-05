#include "spatial_home.h"

namespace CommandItem {

COMMANDITEM SpatialHome::getCommandType() const
{
    return COMMANDITEM::CI_NAV_HOME;
}

std::string SpatialHome::getDescription() const
{
    return "This stores the home location for a vehicle";
}

bool SpatialHome::hasSpatialInfluence() const
{
    return true;
}

AbstractCommandItem* SpatialHome::getClone() const
{
    return (new SpatialHome(*this));
}

void SpatialHome::getClone(AbstractCommandItem** command) const
{
    *command = new SpatialHome(*this);
}

SpatialHome::SpatialHome():
    AbstractCommandItem(0,0), AbstractSpatialPosition()
{

}

SpatialHome::~SpatialHome()
{

}

SpatialHome::SpatialHome(const SpatialHome &obj):
    AbstractCommandItem(obj), AbstractSpatialPosition(obj)
{

}

SpatialHome::SpatialHome(const int &systemOrigin, const int &systemTarget):
    AbstractCommandItem(systemOrigin,systemTarget), AbstractSpatialPosition()
{

}

mace_home_position_t SpatialHome::getMACECommsObject() const
{
    mace_home_position_t homePosition;
    homePosition.latitude = position->getX() * pow(10,7);
    homePosition.longitude = position->getY() * pow(10,7);
    homePosition.altitude = position->getZ() * pow(10,3);
    return homePosition;
}

mace_message_t SpatialHome::getMACEMsg(const uint8_t systemID, const uint8_t compID, const uint8_t chan) const
{
    mace_message_t msg;
    mace_home_position_t homePosition = getMACECommsObject();
    mace_msg_home_position_encode_chan(systemID,compID,chan,&msg,&homePosition);
    return msg;
}

std::ostream& operator<<(std::ostream& os, const SpatialHome& t)
{
    std::stringstream stream;
    stream.precision(6);
    stream << std::fixed << "Spatial Home: "<< t.position->getX() << ", "<< t.position->getY() << ", "<< t.position->getZ() << ".";
    os << stream.str();

    return os;
}

} //end of namespace CommandItem
