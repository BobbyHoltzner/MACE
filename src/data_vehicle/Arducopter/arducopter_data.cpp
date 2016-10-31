#include "arducopter_data.h"

using namespace Data;

ArducopterData::~ArducopterData(){

}

VehicleProtocol ArducopterData::getProtocolDefinition() const
{
    return Data::PROTOCOL_ARDUPILOT;
}
