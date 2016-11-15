#include "vehicle_generic_mavlink.h"

namespace Vehicle_MAVLINK {
VehicleGeneric_MAVLINK::VehicleGeneric_MAVLINK(const int &vehicleID, const VehicleProtocolENUM &vehicleProtocol, const VehicleTypeENUM &vehicleType)
    :VehicleObject(vehicleID,vehicleProtocol,vehicleType)
{

}

void VehicleGeneric_MAVLINK::handleMessage(VehicleMessage message) const
{\

}
}//end of namespace Vehicle_MAVLINK
