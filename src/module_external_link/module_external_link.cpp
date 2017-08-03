#include "module_external_link.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////
///             CONFIGURE
////////////////////////////////////////////////////////////////////////////////////////////////////////

ModuleExternalLink::ModuleExternalLink() :
    m_VehicleDataTopic("vehicleData"),m_MissionDataTopic("vehicleMission"),
    associatedSystemID(254), airborneInstance(true),
    m_CommandController(NULL),m_MissionController(NULL)
{
    m_MissionController = new ExternalLink::MissionController_ExternalLink(this);
    m_CommandController = new ExternalLink::CommandController_ExternalLink(this);
}

ModuleExternalLink::~ModuleExternalLink()
{

}

void ModuleExternalLink::createLog(const int &systemID)
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
    logname = rootPath + kPathSeparator + "logs/ExternalLinkModule" + std::to_string(systemID) + ".txt";
    std::string loggerName = "ExternalLinkModule_" + std::to_string(systemID);
    char logNameArray[loggerName.size()+1];//as 1 char space for null is also required
    strcpy(logNameArray, loggerName.c_str());

    //initiate the logs
    size_t q_size = 8192; //queue size must be power of 2
    spdlog::set_async_mode(q_size,spdlog::async_overflow_policy::discard_log_msg,nullptr,std::chrono::seconds(2));
    mLog = spdlog::basic_logger_mt(logNameArray, logname);
    mLog->set_level(spdlog::level::debug);
}

void ModuleExternalLink::transmitMessage(const mace_message_t &msg)
{
    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
}

///////////////////////////////////////////////////////////////////////////////////////
/// The following are public virtual functions imposed from the Command Controller
/// Interface via callback functionality.
///////////////////////////////////////////////////////////////////////////////////////
void ModuleExternalLink::cbiCommandController_transmitCommand(const mace_command_short_t &cmd)
{
    mace_message_t msg;
    mace_msg_command_short_encode_chan(this->associatedSystemID,0,m_LinkChan,&msg,&cmd);
    transmitMessage(msg);
}

void ModuleExternalLink::cbiCommandController_transmitCommand(const mace_command_long_t &cmd)
{
    mace_message_t msg;
    mace_msg_command_long_encode_chan(this->associatedSystemID,0,m_LinkChan,&msg,&cmd);
    transmitMessage(msg);
}

void ModuleExternalLink::cbiCommandController_CommandACK(const mace_command_ack_t &ack)
{

}

///////////////////////////////////////////////////////////////////////////////////////
/// The following are public virtual functions imposed from the Mission Controller
/// Interface via callback functionality.
///////////////////////////////////////////////////////////////////////////////////////
void ModuleExternalLink::cbiMissionController_TransmitMissionACK(const mace_mission_ack_t &missionACK)
{
    mace_message_t msg;
    mace_msg_mission_ack_encode_chan(this->associatedSystemID,0,m_LinkChan,&msg,&missionACK);
    transmitMessage(msg);
}

void ModuleExternalLink::cbiMissionController_TransmitMissionCount(const mace_mission_count_t &count)
{
    mace_message_t msg;
    mace_msg_mission_count_encode_chan(this->associatedSystemID,0,m_LinkChan,&msg,&count);
    transmitMessage(msg);
}

void ModuleExternalLink::cbiMissionController_TransmitMissionItem(const mace_mission_item_t &item)
{
    mace_message_t msg;
    mace_msg_mission_item_encode_chan(this->associatedSystemID,0,m_LinkChan,&msg,&item);
    transmitMessage(msg);
}

void ModuleExternalLink::cbiMissionController_TransmitMissionReqList(const mace_mission_request_list_t &request)
{
    mace_message_t msg;
    mace_msg_mission_request_list_encode_chan(this->associatedSystemID,0,m_LinkChan,&msg,&request);
    transmitMessage(msg);
}

void ModuleExternalLink::cbiMissionController_TransmitMissionReq(const mace_mission_request_item_t &requestItem)
{
    mace_message_t msg;
    mace_msg_mission_request_item_encode_chan(this->associatedSystemID,0,m_LinkChan,&msg,&requestItem);
    transmitMessage(msg);
}

void ModuleExternalLink::cbiMissionController_TransmitHomeReq(const mace_mission_request_home_t &request)
{
    mace_message_t msg;
    mace_msg_mission_request_home_encode_chan(this->associatedSystemID,0,m_LinkChan,&msg,&request);
    transmitMessage(msg);
}

void ModuleExternalLink::cbiMissionController_ReceviedHome(const CommandItem::SpatialHome &home)
{
    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
        ptr->GVEvents_NewHomePosition(this,home);
    });

    std::shared_ptr<CommandItem::SpatialHome> homePtr = std::make_shared<CommandItem::SpatialHome>(home);
    std::shared_ptr<MissionTopic::MissionHomeTopic> missionTopic = std::make_shared<MissionTopic::MissionHomeTopic>();
    missionTopic->setHome(homePtr);

    MaceCore::TopicDatagram topicDatagram;
    m_MissionDataTopic.SetComponent(missionTopic, topicDatagram);
    //notify listneres of topic
    ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(this, m_MissionDataTopic.Name(), home.getOriginatingSystem(), MaceCore::TIME(), topicDatagram);
    });
}

void ModuleExternalLink::cbiMissionController_ReceivedMission(const MissionItem::MissionList &missionList)
{
    std::cout<<"The external link module now has received the entire mission."<<std::endl;
}

void ModuleExternalLink::cbiMissionController_MissionACK(const mace_mission_ack_t &missionACK)
{
    std::cout<<"The external link module now has received the mission ack."<<std::endl;
}


//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleExternalLink::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
    ptr->Subscribe(this, m_VehicleDataTopic.Name());
    ptr->Subscribe(this, m_MissionDataTopic.Name());
}

//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleExternalLink::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;
    ConfigureMACEStructure(structure);
    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);

    std::shared_ptr<MaceCore::ModuleParameterStructure> moduleSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    moduleSettings->AddTerminalParameters("AirborneInstance", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, true);
    structure.AddNonTerminal("ModuleParameters", moduleSettings, true);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);

}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleExternalLink::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    ConfigureMACEComms(params);

    if(params->HasNonTerminal("ModuleParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> moduleSettings = params->GetNonTerminalValue("ModuleParameters");
        airborneInstance = moduleSettings->GetTerminalValue<bool>("AirborneInstance");
    }

}

//!
//! \brief New Mavlink message received over a link
//! \param linkName Name of link message received over
//! \param msg Message received
//!
void ModuleExternalLink::MACEMessage(const std::string &linkName, const mace_message_t &message)
{
    UNUSED(linkName);
    this->ParseForData(&message);
}



void ModuleExternalLink::HeartbeatInfo(const int &systemID, const mace_heartbeat_t &heartbeatMSG)
{
    if(systemIDMap.find(systemID) == systemIDMap.end())
    {
        m_MissionController->updateIDS(systemID,254);
        m_CommandController->updateIDS(systemID,254);

        //The system has yet to have communicated through this module
        //We therefore have to notify the core that there is a new vehicle
        systemIDMap.insert({systemID,0});
        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
            ptr->ExternalEvent_NewConstructedVehicle(this, systemID);
        });
        //Request_FullDataSync(systemID);
    }

    DataGenericItem::DataGenericItem_Heartbeat heartbeat;
    heartbeat.setAutopilot(static_cast<Data::AutopilotType>(heartbeatMSG.autopilot));
    heartbeat.setCompanion((heartbeatMSG.mace_companion>0)? true : false);
    heartbeat.setProtocol(static_cast<Data::CommsProtocol>(heartbeatMSG.protocol));
    heartbeat.setExecutionState(static_cast<Data::MissionExecutionState>(heartbeatMSG.mission_state));
    heartbeat.setType(static_cast<Data::SystemType>(heartbeatMSG.type));
    //heartbeat.setExecutionState(static_cast<Data::MissionExecutionState>(heartbeatMSG.missionState));
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Heartbeat> ptrHeartbeat = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Heartbeat>(heartbeat);
    PublishVehicleData(systemID,ptrHeartbeat);
}

//Ken Fix This
//void ModuleExternalLink::MACESyncMessage(const std::string &linkName, const int &systemID, const mace_vehicle_sync_t &syncMSG)
//{
//    std::cout<<"External link saw a full data request from remote"<<std::endl;
//    ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsExternalLink* ptr){
//        ptr->Event_ForceVehicleDataSync(this, syncMSG.target_system);
//    });
//}

//!
//! \brief ModuleExternalLink::VehicleCommandMACEACK
//! \param linkName
//! \param systemID
//! \param cmdACK
//!
//Ken Fix This
//void ModuleExternalLink::MACECommandACK(const std::string &linkName, const int &systemID, const mace_command_ack_t &cmdACK)
//{
//    m_CommandController->receivedCommandACK();
//    Data::CommandItemType cmdType = static_cast<Data::CommandItemType>(cmdACK.command);
//    Data::CommandACKType ackType = static_cast<Data::CommandACKType>(cmdACK.result);

//    std::string cmdString = Data::CommandItemTypeToString(cmdType);
//    std::string ackString = Data::CommandACKToString(ackType);

//    std::stringstream ss;
//    ss << "System " << systemID << " " << ackString << " command " << cmdString;
//    std::string newString = ss.str();

//    MaceCore::TopicDatagram topicDatagram;
//    DataGenericItem::DataGenericItem_Text newText;
//    newText.setSeverity(Data::StatusSeverityType::STATUS_INFO);
//    newText.setText(newString);
//    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> ptrStatusText = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Text>(newText);

//    m_VehicleDataTopic.SetComponent(ptrStatusText, topicDatagram);
//    //notify listneres of topic
//    ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
//        ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
//    });

//}

void ModuleExternalLink::PublishVehicleData(const int &systemID, const std::shared_ptr<Data::ITopicComponentDataObject> &component)
{
    //construct datagram
    MaceCore::TopicDatagram topicDatagram;
    m_VehicleDataTopic.SetComponent(component, topicDatagram);
    //notify listneres of topic
    ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
    });
}

///////////////////////////////////////////////////////////////////////////////////////
/// The following are public virtual functions imposed from IModuleCommandExternalLink.
//////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
/// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
/// command and action sequence that accompanies the vheicle. Expect an
/// acknowledgement or an event to take place when calling these items.
////////////////////////////////////////////////////////////////////////////

void ModuleExternalLink::Request_FullDataSync(const int &targetSystem)
{
    std::cout<<"External link saw a full data request from local"<<std::endl;
    mace_message_t msg;
    mace_vehicle_sync_t sync;
    sync.target_system = targetSystem;
    mace_msg_vehicle_sync_encode_chan(associatedSystemID,0,m_LinkChan,&msg,&sync);
    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
}

void ModuleExternalLink::Command_SystemArm(const CommandItem::ActionArm &systemArm)
{
    m_CommandController->setSystemArm(systemArm);
}

void ModuleExternalLink::Command_ChangeSystemMode(const CommandItem::ActionChangeMode &vehicleMode)
{
    UNUSED(vehicleMode);
}

void ModuleExternalLink::Command_VehicleTakeoff(const CommandItem::SpatialTakeoff &vehicleTakeoff)
{
    m_CommandController->setSystemTakeoff(vehicleTakeoff);
}

void ModuleExternalLink::Command_Land(const CommandItem::SpatialLand &vehicleLand)
{
    m_CommandController->setSystemLand(vehicleLand);
}

void ModuleExternalLink::Command_ReturnToLaunch(const CommandItem::SpatialRTL &vehicleRTL)
{
    m_CommandController->setSystemRTL(vehicleRTL);
}

void ModuleExternalLink::Command_MissionState(const CommandItem::ActionMissionCommand &command)
{
    //KEN FIX THIS
    mace_message_t msg = DataCOMMS::Command_MACETOCOMMS::generateMissionCommandMessage(command,m_LinkChan);
    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
}

void ModuleExternalLink::Command_IssueGeneralCommand(const std::shared_ptr<CommandItem::AbstractCommandItem> &command)
{
    UNUSED(command);
}

void ModuleExternalLink::Command_EmitHeartbeat(const CommandItem::SpatialTakeoff &heartbeat)
{

}

/////////////////////////////////////////////////////////////////////////////
/// GENERAL HOME EVENTS: These events are related to establishing or setting
/// a home position. It should be recognized that the first mission item in a
/// mission queue should prepend this position. Just the way ardupilot works.
/////////////////////////////////////////////////////////////////////////////

void ModuleExternalLink::Command_GetHomePosition(const int &vehicleID)
{
    std::cout<<"External link module saw a get home position request"<<std::endl;
    m_MissionController->requestHome(vehicleID);
}

void ModuleExternalLink::Command_SetHomePosition(const CommandItem::SpatialHome &systemHome)
{
    m_CommandController->setHomePosition(systemHome);
}

/////////////////////////////////////////////////////////////////////////
/// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
/// This functionality may be pertinent for vehicles not containing a
/// direct MACE hardware module.
/////////////////////////////////////////////////////////////////////////

void ModuleExternalLink::Command_UploadMission(const MissionItem::MissionList &missionList)
{
    MissionItem::MissionList::MissionListStatus status = missionList.getMissionListStatus();
    int targetSystem = missionList.getVehicleID();
    if(status.state == MissionItem::MissionList::COMPLETE)
    {
        m_MissionController->transmitMission(targetSystem,missionList);
    }
}

void ModuleExternalLink::Command_GetMission(const Data::MissionKey &key)
{
    UNUSED(key);
    m_MissionController->requestMission(key);
}

void ModuleExternalLink::Command_SetCurrentMission(const Data::MissionKey &key)
{
//    mace_message_t msg;
//    mace_mission_set_current_t request;
//    request.
//    request.mission_creator = key.m_creatorID;
//    request.mission_id = key.m_missionID;
//    request.mission_type = (uint8_t)key.m_missionType;
//    request.target_system = key.m_systemID;
//    mace_msg_mission_set_current_encode_chan(associatedSystemID,0,m_LinkChan,&msg,&request);
//    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
}

void ModuleExternalLink::Command_GetCurrentMission(const int &targetSystem)
{
    UNUSED(targetSystem);

}

void ModuleExternalLink::Command_ClearCurrentMission(const int &targetSystem)
{
    UNUSED(targetSystem);
}

void ModuleExternalLink::Command_GetOnboardAuto(const int &targetSystem)
{
    UNUSED(targetSystem);

}

void ModuleExternalLink::Command_ClearOnboardAuto(const int &targetSystem)
{
    UNUSED(targetSystem);
}

void ModuleExternalLink::Command_GetOnboardGuided(const int &targetSystem)
{
    UNUSED(targetSystem);
}

void ModuleExternalLink::Command_ClearOnboardGuided(const int &targetSystem)
{
    UNUSED(targetSystem);
}

void ModuleExternalLink::NewlyAvailableOnboardMission(const Data::MissionKey &key)
{
    mace_new_onboard_mission_t mission;
    mission.mission_creator = key.m_creatorID;
    mission.mission_id = key.m_missionID;
    mission.mission_type = (uint8_t)key.m_missionType;
    mission.mission_system = key.m_systemID;
    mission.mission_state = (uint8_t)key.m_missionState;

    mace_message_t msg;
    mace_msg_new_onboard_mission_encode_chan(associatedSystemID,0,m_LinkChan,&msg,&mission);
    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
}

void ModuleExternalLink::NewlyAvailableHomePosition(const CommandItem::SpatialHome &home)
{
    mace_home_position_t homePos;
    float power = pow(10,7);
    homePos.latitude = home.position.getX() * power;
    homePos.longitude = home.position.getY() * power;
    homePos.altitude = home.position.getZ() * power;
    mace_message_t msg;
    mace_msg_home_position_encode_chan(home.getOriginatingSystem(),0,m_LinkChan,&msg,&homePos);
    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
}

//Ken FIX THIS: I dont know if I should pass the pertinent systemID with the key
void ModuleExternalLink::NewlyAvailableMissionExeState(const Data::MissionKey &key)
{
    MissionItem::MissionList list;
    bool validity = this->getDataObject()->getMissionList(key,list);
    if(validity)
    {
        mace_mission_exe_state_t state;
        Data::MissionExecutionState missionState = list.getMissionExeState();
        state.mission_creator = key.m_creatorID;
        state.mission_id = key.m_missionID;
        state.mission_state = (uint8_t)missionState;
        state.mission_system = key.m_systemID;
        state.mission_type = (uint8_t)key.m_missionType;

        mace_message_t msg;
        mace_msg_mission_exe_state_encode_chan(key.m_systemID,0,m_LinkChan,&msg,&state);
        m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
    }
}

void ModuleExternalLink::NewlyAvailableVehicle(const int &systemID)
{
    if(airborneInstance)
    {
        m_MissionController->updateIDS(254,systemID);
        m_CommandController->updateIDS(254,systemID);
    }
}

//!
//! \brief ModuleExternalLink::NewTopic
//! \param topicName
//! \param senderID
//! \param componentsUpdated
//!
void ModuleExternalLink::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{

    DataCOMMS::Generic_MACETOCOMMS helper;
    if(airborneInstance)
    {
        //In relevance to the external link module, the module when receiving a new topic should pack that up for transmission
        //to other instances of MACE
        //example read of vehicle data
        if(topicName == m_VehicleDataTopic.Name())
        {
            //get latest datagram from mace_data
            MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), senderID);
            //example of how to get data and parse through the components that were updated
            for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
                if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Heartbeat::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Heartbeat> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Heartbeat>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = helper.HeartbeatTopicPTR_MACETOCOMMS(component,senderID,0,m_LinkChan);
                    associatedSystemID = senderID;
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_GPS::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_GPS> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_GPS>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = helper.Generic_MACETOCOMMS::GPSTopicPTR_MACETOCOMMS(component,senderID,0,m_LinkChan);
                    associatedSystemID = senderID;
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_FlightMode::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = helper.Generic_MACETOCOMMS::FlightModeTopicPTR_MACETOCOMMS(component,senderID,0,m_LinkChan);
                    associatedSystemID = senderID;
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_SystemArm::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_SystemArm>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = helper.SystemArmTopicPTR_MACETOCOMMS(component,senderID,0,m_LinkChan);
                    associatedSystemID = senderID;
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataStateTopic::StateAttitudeTopic::Name()) {
                    std::shared_ptr<DataStateTopic::StateAttitudeTopic> component = std::make_shared<DataStateTopic::StateAttitudeTopic>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = DataCOMMS::State_MACETOCOMMS::AttitudeStateFullTopicPTR_MACETOCOMMS(component,senderID,0,m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {
                    std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> component = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = DataCOMMS::State_MACETOCOMMS::GlobalPositionTopicPTR_MACETOCOMMS(component,senderID,0,m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Battery::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Battery> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Battery>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = helper.Generic_MACETOCOMMS::BatteryTopicPTR_MACETOCOMMS(component,senderID,0,m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
                else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_Text::Name()) {
                    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Text> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Text>();
                    m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                    mace_message_t msg = helper.Generic_MACETOCOMMS::TextTopicPTR_MACETOCOMMS(component,senderID,0,m_LinkChan);
                    m_LinkMarshaler->SendMACEMessage<mace_message_t>(m_LinkName, msg);
                }
            }
        }
    }
}

