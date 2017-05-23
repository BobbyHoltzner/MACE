#include "module_vehicle_ardupilot.h"

ModuleVehicleArdupilot::ModuleVehicleArdupilot() :
    ModuleVehicleMAVLINK<DATA_VEHICLE_ARDUPILOT_TYPES>(),
    m_VehicleMission("vehicleMission"), m_AircraftController(NULL)
{

}

std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> ModuleVehicleArdupilot::getArducopterData(const int &systemID)
{
    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData;
    try{
        tmpData = m_ArduPilotData.at(systemID);
    }catch(const std::out_of_range &oor)
    {
        tmpData = std::make_shared<DataARDUPILOT::VehicleObject_ARDUPILOT>(systemID,255,0);
        m_ArduPilotData.insert({systemID,tmpData});

        ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
            ptr->NewConstructedVehicle(this, systemID);
        });

//        std::shared_ptr<Ardupilot_GuidedController> newController = std::make_shared<Ardupilot_GuidedController>(tmpData, m_LinkMarshaler, m_LinkName, m_LinkChan);
//        m_ArdupilotController[systemID] = newController;

//        std::thread *thread = new std::thread([newController]()
//        {
//            newController->start();
//        });
    }

    return tmpData;
}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleVehicleArdupilot::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
    ptr->Subscribe(this, m_VehicleMission.Name());
}

////////////////////////////////////////////////////////////////////////////
/// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
/// command and action sequence that accompanies the vheicle. Expect an
/// acknowledgement or an event to take place when calling these items.
////////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::Command_SystemArm(const CommandItem::ActionArm &command)
{
    int vehicleID = command.getTargetSystem();
    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData = getArducopterData(vehicleID);
    mavlink_message_t msg = tmpData->generateArmMessage(command,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::Command_VehicleTakeoff(const CommandItem::SpatialTakeoff<DataState::StateGlobalPosition> &command)
{
    int vehicleID = command.getTargetSystem();
//    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData = getArducopterData(vehicleID);
//    mavlink_message_t msg = tmpData->generateTakeoffMessage(vehicleTakeoff,m_LinkChan);
//    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData = getArducopterData(vehicleID);
    Ardupilot_TakeoffController* newController = new Ardupilot_TakeoffController(tmpData, m_LinkMarshaler, m_LinkName, m_LinkChan);
    if(command.getPositionFlag())
        newController->initializeTakeoffSequence(command);
    else{
        CommandItem::SpatialTakeoff<DataState::StateGlobalPosition> defaultTakeoff = command;
        newController->initializeTakeoffSequence(defaultTakeoff);
    }

    this->SpinUpController(newController);
}

void ModuleVehicleArdupilot::Command_Land(const CommandItem::SpatialLand<DataState::StateGlobalPosition> &command)
{
    int vehicleID = command.getTargetSystem();
    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData = getArducopterData(vehicleID);
    mavlink_message_t msg = tmpData->generateLandMessage(command,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::Command_ReturnToLaunch(const CommandItem::SpatialRTL &command)
{
    int vehicleID = command.getTargetSystem();
    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData = getArducopterData(vehicleID);
    mavlink_message_t msg = tmpData->generateRTLMessage(command,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::Command_ChangeSystemMode(const CommandItem::ActionChangeMode &command)
{
    int vehicleID = command.getTargetSystem();
    std::string modeString = command.getRequestMode();

    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData = getArducopterData(vehicleID);
    if(tmpData->data->getHearbeatSeen())
    {
        int newFlightMode = tmpData->data->ArdupilotFlightMode.get().getFlightModeFromString(modeString);
        mavlink_message_t msg = tmpData->generateChangeMode(vehicleID,m_LinkChan,newFlightMode);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
    }
}

void ModuleVehicleArdupilot::Command_IssueGeneralCommand(const std::shared_ptr<CommandItem::AbstractCommandItem> &command)
{

}

void ModuleVehicleArdupilot::SpinUpController(Ardupilot_GeneralController *newController) {

    m_AircraftController = newController;
    m_AircraftController->start();
}

void ModuleVehicleArdupilot::SpinDownController() {
    m_AircraftController->terminateObject();
}

/////////////////////////////////////////////////////////////////////////////
/// GENERAL HOME EVENTS: These events are related to establishing or setting
/// a home position. It should be recognized that the first mission item in a
/// mission queue should prepend this position. Just the way ardupilot works.
/////////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::Command_GetHomePosition(const int &vehicleID)
{
    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData = getArducopterData(vehicleID);
    mavlink_message_t msg = tmpData->generateGetHomeMessage(vehicleID,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::Command_SetHomePosition(const CommandItem::SpatialHome &vehicleHome)
{
    int vehicleID = vehicleHome.getTargetSystem();
    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData = getArducopterData(vehicleID);
    mavlink_message_t msg = tmpData->generateSetHomePosition(vehicleHome,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::homePositionUpdated(const CommandItem::SpatialHome &newVehicleHome)
{
    int systemID = newVehicleHome.getGeneratingSystem();
    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData = getArducopterData(systemID);


    if(tmpData->data->vehicleHome.set(newVehicleHome))
    {
        //notify the core of the change
        ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
            ptr->GVEvents_NewHomePosition(this, newVehicleHome);
        });

        std::shared_ptr<CommandItem::SpatialHome> home = std::make_shared<CommandItem::SpatialHome>(newVehicleHome);
        std::shared_ptr<MissionTopic::MissionHomeTopic> homeTopic = std::make_shared<MissionTopic::MissionHomeTopic>();
        homeTopic->setHome(home);
        MaceCore::TopicDatagram topicDatagram;
        m_VehicleMission.SetComponent(homeTopic, topicDatagram);

        //notify listneres of topic
        ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleMission.Name(), systemID, MaceCore::TIME(), topicDatagram);
        });
    }
}

/////////////////////////////////////////////////////////////////////////
/// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
/// This functionality may be pertinent for vehicles not containing a
/// direct MACE hardware module.
/////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::UpdateMissionKey(const Data::MissionKeyChange &key)
{
    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData = getArducopterData(key.oldKey.m_systemID);
    MissionItem::MissionList missionList = tmpData->data->Command_GetCurrentMission(key.oldKey.m_missionType);
    if(missionList.getMissionKey() == key.oldKey)
    {
        missionList.setMissionKey(key.newKey);
        tmpData->data->setCurrentMission(missionList);
    }
}

void ModuleVehicleArdupilot::Command_UploadMission(const MissionItem::MissionList &missionList)
{
    switch(missionList.getCommandType())
    {
    case(Data::MissionType::AUTO): //This case should push the mission directly to the aircraft
    {
        int vehicleID = missionList.getVehicleID();
        std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData = getArducopterData(vehicleID);
        tmpData->data->setProposedMission(missionList);
        mavlink_message_t msg;
        int queueSize = missionList.getQueueSize();

        mavlink_msg_mission_count_pack_chan(255,190,m_LinkChan,&msg,vehicleID,0,queueSize,MAV_MISSION_TYPE_MISSION);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
        break;
    }
    case(Data::MissionType::GUIDED):
    {
        //In these two cases the mission should be carefully considered if we are a module
        //aboard MACE hardware companion package or communicating via ground link
        if(airborneInstance)
        {
            //If we are an airborne instance we can acknowledge this right away
            //This implies we are aboard the aircraft directly communicating with the autopilot
            //Thus higher rate capabilities and request for state are available
        }else{

        }
        break;
    }
    default:
        break;
    }

    //notify the core of the change
    ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
        ptr->EventVehicle_ACKProposedMission(this, missionList.getMissionKey());
    });
}


void ModuleVehicleArdupilot::Command_SetCurrentMission(const Data::MissionKey &key)
{
    UNUSED(key);
}

void ModuleVehicleArdupilot::Command_GetCurrentMission(const int &targetSystem)
{
    UNUSED(targetSystem);
}

void ModuleVehicleArdupilot::Command_GetMission(const Data::MissionKey &key)
{
    UNUSED(key);
}

void ModuleVehicleArdupilot::Command_ClearCurrentMission(const int &targetSystem)
{
    UNUSED(targetSystem);
}

////////////////////////////////////////////////////////////////////////////
/// GENERAL AUTO EVENTS: This is implying for auto mode of the vehicle.
/// This functionality is pertinent for vehicles that may contain a
/// MACE HW module, or, vehicles that have timely or ever updating changes.
////////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::Command_GetOnboardAuto(const int &targetSystem)
{
    mavlink_message_t msg;
    mavlink_msg_mission_request_list_pack_chan(255,190,m_LinkChan,&msg,targetSystem,0,MAV_MISSION_TYPE_MISSION);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::Command_ClearOnboardAuto(const int &targetSystem)
{
    mavlink_message_t msg;
    mavlink_msg_mission_clear_all_pack_chan(255,190,m_LinkChan,&msg,targetSystem,0,MAV_MISSION_TYPE_MISSION);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

/////////////////////////////////////////////////////////////////////////
/// GENERAL GUIDED EVENTS: This is implying for guided mode of the vehicle.
/// This functionality is pertinent for vehicles that may contain a
/// MACE HW module, or, vehicles that have timely or ever updating changes.
/////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::Command_GetOnboardGuided(const int &targetSystem)
{
    UNUSED(targetSystem);
}

void ModuleVehicleArdupilot::Command_ClearOnboardGuided(const int &targetSystem)
{
    UNUSED(targetSystem);
}


void ModuleVehicleArdupilot::VehicleHeartbeatInfo(const std::string &linkName, const int systemID, const mavlink_heartbeat_t &heartbeatMSG)
{
    //MaceCore::TopicDatagram topicDatagram;

    UNUSED(linkName);
    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData = getArducopterData(systemID);
    //The purpose of this module seeing if it is the first heartbeat is to establish initial comms and parameter grabs
    //from the vehicle.
    if(!tmpData->data->getHearbeatSeen())
    {
        mavlink_message_t msg;

        //this is the first time we have seen this heartbeat
        //Let us first turn on the data streams
        mavlink_request_data_stream_t request;
        request.target_system = 0;
        request.target_component = 0;
        request.start_stop = 1;

        request.req_stream_id = 2;
        request.req_message_rate = 1;
        mavlink_msg_request_data_stream_encode_chan(255,190,m_LinkChan,&msg,&request);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);

        request.req_stream_id = 6;
        request.req_message_rate = 3;
        mavlink_msg_request_data_stream_encode_chan(255,190,m_LinkChan,&msg,&request);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);

        request.req_stream_id = 10;
        request.req_message_rate = 5;
        mavlink_msg_request_data_stream_encode_chan(255,190,m_LinkChan,&msg,&request);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);

        request.req_stream_id = 11;
        request.req_message_rate = 2;
        mavlink_msg_request_data_stream_encode_chan(255,190,m_LinkChan,&msg,&request);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);

        tmpData->data->setHeartbeatSeen(true);
        mavlink_msg_mission_request_list_pack_chan(255,190,m_LinkChan,&msg,systemID,0,MAV_MISSION_TYPE_MISSION);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);

    }

    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> rtnVector;

    DataGenericItem::DataGenericItem_Heartbeat heartbeat;
    heartbeat.setAutopilot(Data::AutopilotType::AUTOPILOT_TYPE_ARDUPILOTMEGA);
    heartbeat.setCompaion(true);
    heartbeat.setProtocol(Data::CommsProtocol::COMMS_MAVLINK);
    switch(heartbeatMSG.type)
    {
    case MAV_TYPE_TRICOPTER:
    case MAV_TYPE_QUADROTOR:
    case MAV_TYPE_HEXAROTOR:
    case MAV_TYPE_OCTOROTOR:
        heartbeat.setType(Data::SystemType::SYSTEM_TYPE_QUADROTOR);
        break;
    case MAV_TYPE_FIXED_WING:
        heartbeat.setType(Data::SystemType::SYSTEM_TYPE_GENERIC);
        break;
    default:
        heartbeat.setType(Data::SystemType::SYSTEM_TYPE_GENERIC);
    }

    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Heartbeat> ptrHeartbeat = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Heartbeat>(heartbeat);
    rtnVector.push_back(ptrHeartbeat);

    DataGenericItem::DataGenericItem_SystemArm arm;
    arm.setSystemArm(heartbeatMSG.base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY);
    if(tmpData->data->vehicleArm.set(arm))
    {
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> ptrArm = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_SystemArm>(arm);
        rtnVector.push_back(ptrArm);
    }

    DataARDUPILOT::VehicleFlightMode newDataMode;
    newDataMode.parseMAVLINK(heartbeatMSG);
    newDataMode.getFlightModeString();
    DataGenericItem::DataGenericItem_FlightMode mode;
    mode.setFlightMode(newDataMode.getFlightModeString());
    if(tmpData->data->vehicleMode.set(mode))
    {
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> ptrMode = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>(mode);
        rtnVector.push_back(ptrMode);
    }

    this->PublishVehicleData(systemID, rtnVector);
}

void ModuleVehicleArdupilot::MAVLINKCommandAck(const std::string &linkName, const int systemID, const mavlink_command_ack_t &cmdACK)
{
    UNUSED(linkName);
    UNUSED(systemID);
    if(checkControllerState())
        m_AircraftController->updateCommandACK(cmdACK);

    switch(cmdACK.result)
    {
    case MAV_RESULT_ACCEPTED:
        std::cout<<"MAV result accepted"<<std::endl;
        break;
    case MAV_RESULT_TEMPORARILY_REJECTED:
        std::cout<<"MAV result rejected"<<std::endl;
        break;
    case MAV_RESULT_DENIED:
        std::cout<<"MAV result denied"<<std::endl;
        break;
    case MAV_RESULT_UNSUPPORTED:
        std::cout<<"MAV result unsupported"<<std::endl;
        break;
    case MAV_RESULT_FAILED:
        std::cout<<"MAV result failed"<<std::endl;
        break;
    default:
        std::cout<<"Uknown ack!"<<std::endl;
    }

}

//!
//! \brief New Mavlink message received over a link
//! \param linkName Name of link message received over
//! \param msg Message received
//!
void ModuleVehicleArdupilot::MavlinkMessage(const std::string &linkName, const mavlink_message_t &message)
{
    int systemID = message.sysid;
    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData = getArducopterData(systemID);

    bool wasMissionMSG = ParseMAVLINKMissionMessage(tmpData, linkName, &message);

    if(wasMissionMSG == false){
        //generate topic datagram from given mavlink message
        std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> components = tmpData->parser->ParseForVehicleData(&message);
        //proceed to send components only if there is 1 or more
        if(components.size() > 0)
        {
            //construct datagram
            MaceCore::TopicDatagram topicDatagram;
            for(size_t i = 0 ; i < components.size() ; i++)
            {
                m_VehicleDataTopic.SetComponent(components.at(i), topicDatagram);
                //notify listneres of topic
                ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                    ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
                });
            }
        } //if there is information available
    }
}

void ModuleVehicleArdupilot::PublishVehicleData(const int &systemID, const std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> &components)
{
    if(components.size() > 0)
    {
        //construct datagram
        MaceCore::TopicDatagram topicDatagram;
        for(size_t i = 0 ; i < components.size() ; i++)
        {
            m_VehicleDataTopic.SetComponent(components.at(i), topicDatagram);
            //notify listneres of topic
            ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
            });
        }
    } //if there is information available
}

void ModuleVehicleArdupilot::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    if(topicName == m_VehicleMission.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleMission.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++){
            if(componentsUpdated.at(i) == MissionTopic::MissionItemTopic::Name()) {
                std::shared_ptr<MissionTopic::MissionItemTopic> component = std::make_shared<MissionTopic::MissionItemTopic>();
                m_VehicleMission.GetComponent(component, read_topicDatagram);
            }
            else if(componentsUpdated.at(i) == MissionTopic::MissionListTopic::Name()){
                std::shared_ptr<MissionTopic::MissionListTopic> component = std::make_shared<MissionTopic::MissionListTopic>();
                m_VehicleMission.GetComponent(component, read_topicDatagram);
            }
        }
    }
}

