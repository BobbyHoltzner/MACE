#include "ardupilot_vehicle_object.h"

ArdupilotVehicleObject::ArdupilotVehicleObject(CommsMAVLINK* commsObj, const int &ID, Controllers::MessageModuleTransmissionQueue<mavlink_message_t> *queue):
    MavlinkVehicleObject(commsObj, ID, queue)
{

}
