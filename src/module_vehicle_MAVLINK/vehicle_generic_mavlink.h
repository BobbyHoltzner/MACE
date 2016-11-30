#ifndef VEHICLEGENERIC_MAVLINK_H
#define VEHICLEGENERIC_MAVLINK_H

#include "mace_core/vehicle_object.h"

namespace Vehicle_MAVLINK {

class VehicleGeneric_MAVLINK : public VehicleObject
{
public:
    VehicleGeneric_MAVLINK(const int &vehicleID, const VehicleProtocolENUM &vehicleProtocol, const VehicleTypeENUM &vehicleType);

    virtual void handleMessage(VehicleMessage message) const;
};
}//end of namespace Vehicle_MAVLINK
#endif // VEHICLEGENERIC_MAVLINK_H
