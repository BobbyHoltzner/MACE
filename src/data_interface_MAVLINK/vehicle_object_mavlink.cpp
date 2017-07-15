#include "vehicle_object_mavlink.h"

namespace DataInterface_MAVLINK {


void VehicleObject_MAVLINK::async_example()
{
    std::string logname = "";
    char* MACEPath = getenv("MACE_ROOT");

    const char kPathSeparator =
    #ifdef _WIN32
                                '\\';
    #else
                                '/';
    #endif

    std::string rootPath(MACEPath);
    std::cout << "The current MACE_ROOT path is: " << rootPath << std::endl;
    logname = rootPath + kPathSeparator + "logs/testFileInterface.txt";

    //initiate the logs
    size_t q_size = 8192; //queue size must be power of 2
    spdlog::set_async_mode(q_size,spdlog::async_overflow_policy::block_retry,nullptr,std::chrono::seconds(2));
    spdlog::basic_logger_mt("async_file_logger", logname);
}

VehicleObject_MAVLINK::VehicleObject_MAVLINK(const int &vehicleID, const int &transmittingID):
    m_CB(NULL), missionController(NULL), command(NULL), mission(NULL), state(NULL),
    systemID(vehicleID), commandID(transmittingID),
    m_LinkMarshaler(NULL), m_LinkName(""), m_LinkChan(0)
{
    async_example();

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

    //do not delete this object as we did not create it
    m_LinkMarshaler = NULL;
}

void VehicleObject_MAVLINK::updateCommsInfo(Comms::CommsMarshaler *commsMarshaler, const std::string &linkName, const uint8_t &linkChan)
{
    m_LinkMarshaler = commsMarshaler;
    m_LinkName = linkName;
    m_LinkChan = linkChan;

    mavlink_message_t msg;
    mavlink_request_data_stream_t request;

    request.target_system = 0;
    request.target_component = 0;
    request.start_stop = 1;

    request.req_stream_id = 2;
    request.req_message_rate = 1;
    mavlink_msg_request_data_stream_encode_chan(commandID,190,m_LinkChan,&msg,&request);
    transmitMessage(msg);

    request.req_stream_id = 6;
    request.req_message_rate = 3;
    mavlink_msg_request_data_stream_encode_chan(commandID,190,m_LinkChan,&msg,&request);
    transmitMessage(msg);

    request.req_stream_id = 10;
    request.req_message_rate = 5;
    mavlink_msg_request_data_stream_encode_chan(commandID,190,m_LinkChan,&msg,&request);
    transmitMessage(msg);

    request.req_stream_id = 11;
    request.req_message_rate = 2;
    mavlink_msg_request_data_stream_encode_chan(commandID,190,m_LinkChan,&msg,&request);
    transmitMessage(msg);
}

void VehicleObject_MAVLINK::transmitMessage(const mavlink_message_t &msg)
{
    if(m_LinkMarshaler)
    {
        m_LinkMarshaler->SendMAVMessage<mavlink_message_t>(m_LinkName, msg);
    }
}

void VehicleObject_MAVLINK::transmitCommandLong(const mavlink_command_long_t &cmd)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_encode_chan(commandID,0,m_LinkChan,&msg,&cmd);
    transmitMessage(msg);
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
    mission->home.set(home);
    if(m_CB)
        m_CB->cbi_VehicleHome(this->systemID,home);
}

void VehicleObject_MAVLINK::cbiMissionController_ReceivedMission(const MissionItem::MissionList &missionList)
{
    mission->setCurrentMission(missionList);
    if(m_CB)
        m_CB->cbi_VehicleMission(this->systemID,missionList);
}

void VehicleObject_MAVLINK::cbiMissionController_MissionACK(const mavlink_mission_ack_t &missionACK)
{

}
} //end of namespace DataInterface_MAVLINK

