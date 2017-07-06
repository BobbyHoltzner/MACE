#include "vehicle_object_mavlink.h"

namespace DataInterface_MAVLINK {

VehicleObject_MAVLINK::VehicleObject_MAVLINK(const int &vehicleID, const int &transmittingID):
    systemID(vehicleID), commandID(transmittingID)
{
    command = new CommandInterface_MAVLINK(systemID, 0);
    command->connectCallback_CommandLong(VehicleObject_MAVLINK::staticCallbackCMDLongFunction, this);

    mission = new MissionData_MAVLINK();
    state = new StateData_MAVLINK();
}

void VehicleObject_MAVLINK::updateCommsInfo(Comms::CommsMarshaler *commsMarshaler, const std::string &linkName, const uint8_t &linkChan)
{
    m_LinkMarshaler = commsMarshaler;
    m_LinkName = linkName;
    m_LinkChan = linkChan;
}

void VehicleObject_MAVLINK::transmitMessage(const mavlink_message_t &msg)
{
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void VehicleObject_MAVLINK::transmitCommandLong(const mavlink_command_long_t &cmd)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_encode_chan(commandID,0,m_LinkChan,&msg,&cmd);
    transmitMessage(msg);
}

} //end of namespace DataInterface_MAVLINK

