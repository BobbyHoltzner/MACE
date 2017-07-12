#include "vehicle_object_mavlink.h"

namespace DataInterface_MAVLINK {

VehicleObject_MAVLINK::VehicleObject_MAVLINK(const int &vehicleID, const int &transmittingID):
    systemID(vehicleID), commandID(transmittingID),
    m_LinkMarshaler(NULL), m_LinkName(""), m_LinkChan(0),
    command(NULL), missionController(NULL), mission(NULL), state(NULL)
{
    command = new CommandInterface_MAVLINK(systemID, 0);
    command->connectCallback_CommandLong(VehicleObject_MAVLINK::staticCallbackCMDLongFunction, this);

    missionController = new MissionController_MAVLINK(2,0);
    missionController->connectCallback(this);

    mission = new MissionData_MAVLINK();
    state = new StateData_MAVLINK();
    state->connectCallback_State(VehicleObject_MAVLINK::staticCallbackState, this);
}

VehicleObject_MAVLINK::~VehicleObject_MAVLINK()
{
    delete command;
    command = NULL;

    delete mission;
    mission = NULL;

    delete state;
    state = NULL;

    delete m_LinkMarshaler;
    m_LinkMarshaler = NULL;
}

void VehicleObject_MAVLINK::updateCommsInfo(Comms::CommsMarshaler *commsMarshaler, const std::string &linkName, const uint8_t &linkChan)
{
    m_LinkMarshaler = commsMarshaler;
    m_LinkName = linkName;
    m_LinkChan = linkChan;
}

void VehicleObject_MAVLINK::transmitMessage(const mavlink_message_t &msg)
{
    if(m_LinkMarshaler)
    {
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
    }
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

void VehicleObject_MAVLINK::cbiMissionController_TransmitMissionCount(const mavlink_mission_count_t &count)
{
    mavlink_message_t msg;
    mavlink_msg_mission_count_encode_chan(commandID,190,m_LinkChan,&msg,&count);
    transmitMessage(msg);
}

void VehicleObject_MAVLINK::cbiMissionController_TransmitMissionItem(const mavlink_mission_item_t &item)
{
    mavlink_message_t msg;
    mavlink_msg_mission_item_encode_chan(commandID,190,m_LinkChan,&msg,&item);
    transmitMessage(msg);
}

void VehicleObject_MAVLINK::cbiMissionController_TransmitMissionReqList(const mavlink_mission_request_list_t &request)
{
    mavlink_message_t msg;
    mavlink_msg_mission_request_list_encode_chan(commandID,190,m_LinkChan,&msg,&request);
    transmitMessage(msg);
}

void VehicleObject_MAVLINK::cbiMissionController_TransmitMissionReq(const mavlink_mission_request_t &request)
{
    mavlink_message_t msg;
    mavlink_msg_mission_request_encode_chan(commandID,190,m_LinkChan,&msg,&request);
    transmitMessage(msg);
}

void VehicleObject_MAVLINK::cbiMissionController_ReceviedHome(const CommandItem::SpatialHome &home)
{

}

void VehicleObject_MAVLINK::cbiMissionController_ReceivedMission(const MissionItem::MissionList &mission)
{

}
} //end of namespace DataInterface_MAVLINK

