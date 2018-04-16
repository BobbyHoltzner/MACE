#ifndef ARDUPILOT_VEHICLE_OBJECT_H
#define ARDUPILOT_VEHICLE_OBJECT_H

#include "module_vehicle_MAVLINK/vehicle_object/mavlink_vehicle_object.h"

#include "ardupilot_component_flight_mode.h"

class ArdupilotVehicleObject : public MavlinkVehicleObject
{
public:
    ArdupilotVehicleObject(CommsMAVLINK* commsObj, const int &ID = 1);

public:
    ARDUPILOTComponent_FlightMode ardupilotMode;
};

#endif // ARDUPILOT_VEHICLE_OBJECT_H
