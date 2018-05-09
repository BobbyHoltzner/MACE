#include "ardupilot_vehicle_object.h"

ArdupilotVehicleObject::ArdupilotVehicleObject(CommsMAVLINK* commsObj, const int &ID):
    MavlinkVehicleObject(commsObj, ID)
{

}
