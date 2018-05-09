#include "mavlink_vehicle_object.h"

MavlinkVehicleObject::MavlinkVehicleObject(CommsMAVLINK *commsObj, const int &ID):
    m_CB(nullptr), mavlinkID(ID)
{
    this->commsLink = commsObj;

    controllerQueue = new Controllers::MessageModuleTransmissionQueue<mavlink_message_t>(2000, 3);
    state = new StateData_MAVLINK();
}

int MavlinkVehicleObject::getMAVLINKID() const
{
    return this->mavlinkID;
}

CommsMAVLINK* MavlinkVehicleObject::getCommsObject() const
{
    return this->commsLink;
}
