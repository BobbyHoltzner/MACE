#include "vehicle_object_mavlink.h"

namespace DataInterface_MAVLINK {

VehicleObject_MAVLINK::VehicleObject_MAVLINK(const int &vehicleID, const int &transmittingID):
    systemID(vehicleID), commandID(transmittingID)
{
    command = new CommandInterface_MAVLINK(systemID, 0);
    command->connectCallback_CommandLong(VehicleObject_MAVLINK::staticCallbackCMDLongFunction, this);
}

} //end of namespace DataInterface_MAVLINK

