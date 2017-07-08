#include "vehicle_object_mavlink.h"

namespace DataInterface_MAVLINK {

VehicleObject_MAVLINK::VehicleObject_MAVLINK(const int &vehicleID, const int &transmittingID):
    systemID(vehicleID), commandID(transmittingID)
{
    command = new CommandInterface_MAVLINK(systemID, 0);
    command->connectCallback_CommandLong(VehicleObject_MAVLINK::staticCallbackCMDLongFunction, this);

    missionController = new MissionController_MAVLINK();
    missionController->connectCallback_TransmitMSG(VehicleObject_MAVLINK::staticCallbackTransmitMissionMSG, this);

    mission = new MissionData_MAVLINK();
    state = new StateData_MAVLINK();
    state->connectCallback_State(VehicleObject_MAVLINK::staticCallbackState, this);
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

void VehicleObject_MAVLINK::receiveCommand(const DataState::StateGlobalPosition &pos)
{
    std::cout<<"I have received a position"<<std::endl;
}

} //end of namespace DataInterface_MAVLINK

