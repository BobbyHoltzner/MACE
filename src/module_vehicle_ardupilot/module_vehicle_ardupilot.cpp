#include "module_vehicle_ardupilot.h"
#include <functional>

#include "module_generic_MAVLINK/controllers/controller_mavlink_generic_set.h"

#include "mace_core/i_module_events_general.h"

template <typename T>
T CopyCommandAndInsertTarget(const CommandItem::AbstractCommandItem &item, int targetSystem)
{
    T cpy((T&)item);
    cpy.setTargetSystem(targetSystem);
    return cpy;
}


//ModuleVehicleArdupilot::ModuleVehicleArdupilot() :
//    ModuleVehicleMAVLINK<DATA_VEHICLE_ARDUPILOT_TYPES>(),
//    m_VehicleMissionTopic("vehicleMission"), m_AircraftController(NULL), vehicleData(NULL)
ModuleVehicleArdupilot::ModuleVehicleArdupilot() :
    ModuleVehicleMAVLINK<>(),
    vehicleData(nullptr), stateMachine(nullptr)
{


}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleVehicleArdupilot::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    ModuleVehicleMAVLINK::ConfigureModule(params);
}

//!
//! \brief MissionAcknowledgement Generate acknowledgement based on mission result
//! \param missionResult Mission result
//! \param publishResult Acknowledgement to publish out
//!
void ModuleVehicleArdupilot::createLog(const int &systemID)
{
    std::string logname = this->loggingPath + "/VehicleModule_" + std::to_string(systemID) + ".txt";
    std::string loggerName = "VehicleModule_" + std::to_string(systemID);
    char logNameArray[loggerName.size()+1];//as 1 char space for null is also required
    strcpy(logNameArray, loggerName.c_str());

    //initiate the logs
    size_t q_size = 8192; //queue size must be power of 2
    spdlog::set_async_mode(q_size,spdlog::async_overflow_policy::discard_log_msg,nullptr,std::chrono::seconds(2));
    this->mLogs = spdlog::basic_logger_mt(logNameArray, logname);
    mLogs->set_level(spdlog::level::debug);
}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleVehicleArdupilot::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
    ptr->Subscribe(this, m_VehicleMissionTopic.Name());

    ptr->Subscribe(this, this->m_VehicleTopics.m_CommandSystemMode.Name());
}

////////////////////////////////////////////////////////////////////////////
/// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
/// command and action sequence that accompanies the vheicle. Expect an
/// acknowledgement or an event to take place when calling these items.
////////////////////////////////////////////////////////////////////////////

//!
//! \brief Request_FullDataSync Request all data from all systems
//! \param targetSystem Destination of the data dump receiver
//!
void ModuleVehicleArdupilot::Request_FullDataSync(const int &targetSystem, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> objectData = vehicleData->state->GetTopicData();
    this->PublishVehicleData(targetSystem,objectData);
    //vehicleData->m_MissionController->requestMission();
}

//!
//! \brief Command_SystemArm Command an ARM/DISARM action
//! \param command ARM/DISARM command
//! \param sender Generating system
//!
void ModuleVehicleArdupilot::Command_SystemArm(const CommandItem::ActionArm &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    //Temporary solution to solve boadcasting until rework of commands can be done
    CommandItem::ActionArm commandWithTarget = CopyCommandAndInsertTarget<CommandItem::ActionArm>(command, this->GetAttachedMavlinkEntity());

    std::stringstream buffer;
    buffer << commandWithTarget;

    mLogs->debug("Receieved a command system arm.");
    mLogs->info(buffer.str());
    ardupilot::state::AbstractStateArdupilot* currentOuterState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentOuterState());
    currentOuterState->handleCommand(&command);

    ProgressStateMachineStates();
}

//!
//! \brief Command_VehicleTakeoff Command a takeoff action
//! \param command Takeoff altitude and location
//! \param sender Generating system
//!
void ModuleVehicleArdupilot::Command_VehicleTakeoff(const CommandItem::SpatialTakeoff &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    //Temporary solution to solve boadcasting until rework of commands can be done
    CommandItem::SpatialTakeoff commandWithTarget = CopyCommandAndInsertTarget<CommandItem::SpatialTakeoff>(command, this->GetAttachedMavlinkEntity());

    std::stringstream buffer;
    buffer << commandWithTarget;

    mLogs->debug("Receieved a command takeoff.");
    mLogs->info(buffer.str());

    ardupilot::state::AbstractStateArdupilot* currentOuterState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentOuterState());
    currentOuterState->handleCommand(&commandWithTarget);
    ProgressStateMachineStates();
}

//!
//! \brief Command_Land Command a LAND action
//! \param command Land command
//! \param sender Generating system
//!
void ModuleVehicleArdupilot::Command_Land(const CommandItem::SpatialLand &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    //Temporary solution to solve boadcasting until rework of commands can be done
    CommandItem::SpatialLand commandWithTarget = CopyCommandAndInsertTarget<CommandItem::SpatialLand>(command, this->GetAttachedMavlinkEntity());

    std::stringstream buffer;
    buffer << commandWithTarget;

    mLogs->debug("Receieved a command to land.");
    mLogs->info(buffer.str());

    ardupilot::state::AbstractStateArdupilot* currentOuterState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentOuterState());
    currentOuterState->handleCommand(&commandWithTarget);
    ProgressStateMachineStates();
}

//!
//! \brief Command_ReturnToLaunch command a return to launch action
//! \param command RTL command
//! \param sender Generating system
//!
void ModuleVehicleArdupilot::Command_ReturnToLaunch(const CommandItem::SpatialRTL &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    //Temporary solution to solve boadcasting until rework of commands can be done
    CommandItem::SpatialRTL commandWithTarget = CopyCommandAndInsertTarget<CommandItem::SpatialRTL>(command, this->GetAttachedMavlinkEntity());

    mLogs->debug("Receieved a command RTL.");

    ardupilot::state::AbstractStateArdupilot* currentOuterState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentOuterState());
    currentOuterState->handleCommand(&commandWithTarget);
    ProgressStateMachineStates();
}

//!
//! \brief Command_MissionState Command a mission state request
//! \param command Mission state request command
//! \param sender Generating system
//!
void ModuleVehicleArdupilot::Command_MissionState(const CommandItem::ActionMissionCommand &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    //Temporary solution to solve boadcasting until rework of commands can be done
    CommandItem::ActionMissionCommand commandWithTarget = CopyCommandAndInsertTarget<CommandItem::ActionMissionCommand>(command, this->GetAttachedMavlinkEntity());

    mLogs->debug("Receieved a command to change mission state.");

    int systemID = commandWithTarget.getTargetSystem();

    if((vehicleData) && (vehicleData->getMAVLINKID() == systemID))
    {
        ardupilot::state::AbstractStateArdupilot* currentOuterState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentOuterState());
        currentOuterState->handleCommand(&commandWithTarget);
        ProgressStateMachineStates();
    }
}

//!
//! \brief Command_ChangeSystemMode Command a system mode change
//! \param command Change mode command
//! \param sender Generating system
//!
void ModuleVehicleArdupilot::Command_ChangeSystemMode(const CommandItem::ActionChangeMode &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    //Temporary solution to solve boadcasting until rework of commands can be done
    CommandItem::ActionChangeMode commandWithTarget = CopyCommandAndInsertTarget<CommandItem::ActionChangeMode>(command, this->GetAttachedMavlinkEntity());

    std::stringstream buffer;
    buffer << commandWithTarget;

    mLogs->debug("Receieved a command to change the mode.");
    mLogs->info(buffer.str());

    ardupilot::state::AbstractStateArdupilot* outerState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentOuterState());
    outerState->handleCommand(&commandWithTarget);
    ProgressStateMachineStates();
}

//!
//! \brief Command_IssueGeneralCommand Command a general command
//! \param command General command
//!
void ModuleVehicleArdupilot::Command_IssueGeneralCommand(const std::shared_ptr<CommandItem::AbstractCommandItem> &command)
{

}

/////////////////////////////////////////////////////////////////////////////
/// GENERAL HOME EVENTS: These events are related to establishing or setting
/// a home position. It should be recognized that the first mission item in a
/// mission queue should prepend this position. Just the way ardupilot works.
/////////////////////////////////////////////////////////////////////////////

//!
//! \brief Command_GetHomePosition Request a vehicle's home position
//! \param vehicleID Vehicle ID corresponding to the home position
//!
void ModuleVehicleArdupilot::Command_GetHomePosition(const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
//    if((vehicleData) && (vehicleData->getSystemID() == vehicleID))
//        vehicleData->command->getSystemHome();
}

//!
//! \brief Command_SetHomePosition Set a vehicle's home position
//! \param vehicleHome Vehicle home data
//!
void ModuleVehicleArdupilot::Command_SetHomePosition(const CommandItem::SpatialHome &vehicleHome, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    std::stringstream buffer;
    buffer << vehicleHome;

    mLogs->debug("Receieved a command to home position.");
    mLogs->info(buffer.str());

    ardupilot::state::AbstractStateArdupilot* currentOuterState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentOuterState());
    currentOuterState->handleCommand(&vehicleHome);
    ProgressStateMachineStates();
}


/////////////////////////////////////////////////////////////////////////
/// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
/// This functionality may be pertinent for vehicles not containing a
/// direct MACE hardware module.
/////////////////////////////////////////////////////////////////////////

//!
//! \brief UpdateMissionKey Update the current mission's key
//! \param key New mission key
//!
void ModuleVehicleArdupilot::UpdateMissionKey(const MissionItem::MissionKeyChange &key)
{
//    std::shared_ptr<DataARDUPILOT::VehicleObject_ARDUPILOT> tmpData = getArducopterData(key.oldKey.m_systemID);
//    MissionItem::MissionList missionList = tmpData->data->Command_GetCurrentMission(key.oldKey.m_missionType);
//    if(missionList.getMissionKey() == key.oldKey)
//    {
//        missionList.setMissionKey(key.newKey);
//        tmpData->data->setCurrentMission(missionList);
//    }
}

//!
//! \brief Command_UploadMission Command a mission upload
//! \param missionList Mission list to upload
//!
void ModuleVehicleArdupilot::Command_UploadMission(const MissionItem::MissionList &missionList)
{
    std::stringstream buffer;
    buffer << missionList;

    mLogs->info("Vehicle module has been told to upload a mission.");
    mLogs->info(buffer.str());

    switch(missionList.getMissionType())
    {
    case(MissionItem::MISSIONTYPE::AUTO): //This case should push the mission directly to the aircraft
    {
//        if(vehicleData)
//            vehicleData->m_MissionController->transmitMission(missionList);
        break;
    }
    case(MissionItem::MISSIONTYPE::GUIDED):
    {
        //In these two cases the mission should be carefully considered if we are a module
        //aboard MACE hardware companion package or communicating via ground link
        if(airborneInstance)
        {
            //If we are an airborne instance we can acknowledge this right away
            //This implies we are aboard the aircraft directly communicating with the autopilot
            //Thus higher rate capabilities and request for state are available
        }else{
            //since we are not airborne and someone has requested a guided mission, we will assume that
            //we
        }
        break;
    }
    default:
        break;
    }
}

//!
//! \brief Command_SetCurrentMission Issue a set current mission command
//! \param key Mission key to set as current mission
//!
void ModuleVehicleArdupilot::Command_SetCurrentMission(const MissionItem::MissionKey &key)
{
    UNUSED(key);
}

//!
//! \brief Command_GetCurrentMission Issue a get current mission command
//! \param targetSystem System asking for the current mission
//!
void ModuleVehicleArdupilot::Command_GetCurrentMission(const int &targetSystem)
{
//    mavlink_message_t msg;
//    mavlink_msg_mission_request_list_pack_chan(255,190,m_LinkChan,&msg,targetSystem,0,MAV_MISSION_TYPE_MISSION);
//    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

//!
//! \brief Command_GetMission Request a mission by mission key
//! \param key Mission key
//! \param sender System asking for the mission
//!
void ModuleVehicleArdupilot::Command_GetMission(const MissionItem::MissionKey &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender)
{
    UNUSED(key);
}

//!
//! \brief Command_ClearCurrentMission Clear the current mission
//! \param targetSystem System asking for mission cleared
//!
void ModuleVehicleArdupilot::Command_ClearCurrentMission(const int &targetSystem)
{
    UNUSED(targetSystem);
}

////////////////////////////////////////////////////////////////////////////
/// GENERAL AUTO EVENTS: This is implying for auto mode of the vehicle.
/// This functionality is pertinent for vehicles that may contain a
/// MACE HW module, or, vehicles that have timely or ever updating changes.
////////////////////////////////////////////////////////////////////////////

//!
//! \brief Command_GetOnboardAuto Request the current onboard auto mission and state
//! \param targetSystem System asking for auto info
//!
void ModuleVehicleArdupilot::Command_GetOnboardAuto(const int &targetSystem)
{
//    mavlink_message_t msg;
//    mavlink_msg_mission_request_list_pack_chan(255,190,m_LinkChan,&msg,targetSystem,0,MAV_MISSION_TYPE_MISSION);
//    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

//!
//! \brief Command_ClearOnboardAuto Clear the current onboard auto mission and state
//! \param targetSystem System to clear auto info
//!
void ModuleVehicleArdupilot::Command_ClearOnboardAuto(const int &targetSystem)
{
//    mavlink_message_t msg;
//    mavlink_msg_mission_clear_all_pack_chan(255,190,m_LinkChan,&msg,targetSystem,0,MAV_MISSION_TYPE_MISSION);
//    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

/////////////////////////////////////////////////////////////////////////
/// GENERAL GUIDED EVENTS: This is implying for guided mode of the vehicle.
/// This functionality is pertinent for vehicles that may contain a
/// MACE HW module, or, vehicles that have timely or ever updating changes.
/////////////////////////////////////////////////////////////////////////

//!
//! \brief Command_GetOnboardGuided Request the current onboard guided state
//! \param targetSystem System asking for guided info
//!
void ModuleVehicleArdupilot::Command_GetOnboardGuided(const int &targetSystem)
{
    UNUSED(targetSystem);
}

//!
//! \brief Command_ClearOnboardGuided Clear the current onboard guided state
//! \param targetSystem System to clear guided state
//!
void ModuleVehicleArdupilot::Command_ClearOnboardGuided(const int &targetSystem)
{
    UNUSED(targetSystem);
}

//!
//! \brief New Mavlink message received over a link
//! \param linkName Name of link message received over
//! \param msg Message received
//!
bool ModuleVehicleArdupilot::MavlinkMessage(const std::string &linkName, const mavlink_message_t &message)
{
    bool consumed = false;

    //this is necessary because if we have yet to have received a vehicle heartbeat,
    //we have yet to form the vehicle object and accompanying state machine
    if(vehicleData)
    {
        MavlinkEntityKey sender = message.sysid;
        consumed = m_MissionController->ReceiveMessage(&message, sender);

        if(!consumed)
            consumed = ModuleVehicleMAVLINK::MavlinkMessage(linkName, message);

        if(!consumed)
        {
            //ardupilot::state::AbstractStateArdupilot* currentOuterState = static_cast<ardupilot::state::AbstractStateArdupilot*>(stateMachine->getCurrentOuterState());
            //consumed = currentOuterState->handleMAVLINKMessage(message);
            consumed = vehicleData->handleMAVLINKMessage(message);
            if(!consumed)
                consumed = vehicleData->parseMessage(&message);
        }
        ProgressStateMachineStates();
    }

    return consumed;
}

//!
//! \brief VehicleHeartbeatInfo Heartbeat message from vehicle
//! \param linkName Comms link name
//! \param systemID Vehicle ID generating heartbeat
//! \param heartbeatMSG Heartbeat message
//!
void ModuleVehicleArdupilot::VehicleHeartbeatInfo(const std::string &linkName, const int &systemID, const mavlink_heartbeat_t &heartbeatMSG)
{
    UNUSED(linkName);
    if(vehicleData == nullptr)
    {
        MavlinkEntityKey key = systemID;
        SetAttachedMavlinkEntity(key);

        createLog(systemID);
        //this is the first time we have seen this heartbeat or the data was destroyed for some reason
        vehicleData = std::make_shared<ArdupilotVehicleObject>(this, this->GetCharacteristic(), systemID);
        vehicleData->connectCallback(this);
        vehicleData->connectTargetCallback(ModuleVehicleArdupilot::staticCallbackFunction_VehicleTarget, this);

        //vehicleData->updateCommsInfo(m_LinkMarshaler,m_LinkName,m_LinkChan);


        //create "stateless" mission controller
        TransmitQueue<mavlink_message_t, int> *queue = new TransmitQueue<mavlink_message_t, int>();
        m_MissionController = new MAVLINKVehicleControllers::ControllerMission(vehicleData.get(), queue, m_LinkChan);
        m_MissionController->setLambda_DataReceived([this](const void* key, const std::shared_ptr<MAVLINKVehicleControllers::MissionDownloadResult> &data){

            //////////////////////////////
            ///Update about Home position
            CommandItem::SpatialHome home = std::get<0>(*data);
            vehicleData->mission->vehicleHomePosition.set(home);
            this->cbi_VehicleHome(home.getOriginatingSystem(),home);
            //notify the core of the change
//            ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
//                ptr->GVEvents_NewHomePosition(this, home);
//            });

//            std::shared_ptr<CommandItem::SpatialHome> ptrHome = std::make_shared<CommandItem::SpatialHome>(home);
//            std::shared_ptr<MissionTopic::MissionHomeTopic> homeTopic = std::make_shared<MissionTopic::MissionHomeTopic>();
//            homeTopic->setHome(ptrHome);

//            this->cbi_VehicleMissionData(this->GetCharacteristic().ID, homeTopic);



            //////////////////////////////
            ///Update about mission list
            MissionItem::MissionList missionList = std::get<1>(*data);
            missionList.setVehicleID(this->GetAttachedMavlinkEntity());
            vehicleData->mission->currentAutoMission.set(missionList);
            this->cbi_VehicleMission(missionList.getVehicleID(),missionList);

//            //This function shall update the local MACE CORE instance of the mission
//            ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
//                ptr->EventVehicle_NewOnboardVehicleMission(this, missionList);
//            });

//            //We should update all listeners
//            std::shared_ptr<MissionTopic::MissionListTopic> missionTopic = std::make_shared<MissionTopic::MissionListTopic>(missionList);
//            this->cbi_VehicleMissionData(this->GetCharacteristic().ID, missionTopic);
        });
        m_MissionController->AddLambda_Finished(this, [](const bool completed, const uint8_t code){
            printf("Mission Completed");
        });

        //reqrest the missions on the vehicle
        MaceCore::ModuleCharacteristic moduleCharacterstic = this->GetCharacteristic();
        m_MissionController->GetMissions(systemID);

        m_PublicVehicleID = systemID;

        ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
            ptr->Events_NewVehicle(this, systemID, moduleCharacterstic);
        });


        //vehicleData->m_MissionController->requestMission();

        if(stateMachine)
        {
            delete stateMachine;
            stateMachine = nullptr;
        }

        stateMachine = new hsm::StateMachine();
        stateMachine->Initialize<ardupilot::state::State_Unknown>(vehicleData.get());
    }


    std::string currentFlightMode = vehicleData->ardupilotMode.parseMAVLINK(heartbeatMSG);
    DataGenericItem::DataGenericItem_FlightMode flightMode;
    flightMode.setFlightMode(currentFlightMode);
    if(vehicleData->state->vehicleMode.set(flightMode))
    {
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> ptrFlightMode = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>(flightMode);
        this->cbi_VehicleStateData(systemID, ptrFlightMode);
    }

    DataGenericItem::DataGenericItem_SystemArm arm;
    arm.setSystemArm(heartbeatMSG.base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY);
    if(vehicleData->state->vehicleArm.set(arm))
    {
        std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_SystemArm> ptrArm = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_SystemArm>(arm);
        ModuleVehicleMAVLINK::cbi_VehicleStateData(systemID, ptrArm);
    }

    DataGenericItem::DataGenericItem_Heartbeat heartbeat;
    heartbeat.setAutopilot(Data::AutopilotType::AUTOPILOT_TYPE_ARDUPILOTMEGA);
    heartbeat.setCompanion(this->airborneInstance);
    heartbeat.setProtocol(Data::CommsProtocol::COMMS_MAVLINK);
    heartbeat.setMavlinkID(systemID);

    switch(heartbeatMSG.type)
    {
    case MAV_TYPE_TRICOPTER:
    case MAV_TYPE_QUADROTOR:
    case MAV_TYPE_HEXAROTOR:
    case MAV_TYPE_OCTOROTOR:
        heartbeat.setType(Data::SystemType::SYSTEM_TYPE_QUADROTOR);
        break;
    case MAV_TYPE_FIXED_WING:
        heartbeat.setType(Data::SystemType::SYSTEM_TYPE_FIXED_WING);
        break;
    default:
        heartbeat.setType(Data::SystemType::SYSTEM_TYPE_GENERIC);
    }
    vehicleData->state->vehicleHeartbeat.set(heartbeat);
    std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_Heartbeat> ptrHeartbeat = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_Heartbeat>(heartbeat);
    ModuleVehicleMAVLINK::cbi_VehicleStateData(systemID,ptrHeartbeat);

    ProgressStateMachineStates();
}

//!
//! \brief PublishVehicleData Parent publisher for vehicle data
//! \param systemID Vehicle ID generating vehicle data
//! \param components Data components to publish
//!
void ModuleVehicleArdupilot::PublishVehicleData(const int &systemID, const std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> &components)
{
    if(components.size() > 0)
    {
        //construct datagram
        MaceCore::TopicDatagram topicDatagram;
        for(size_t i = 0 ; i < components.size() ; i++)
        {
            m_VehicleDataTopic.SetComponent(components.at(i), topicDatagram);
            //notify listeners of topic
            ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
            });

        }
    } //if there is information available
}

//!
//! \brief New non-spooled topic given
//!
//! NonSpooled topics send their data immediatly.
//! \param topicName Name of stopic
//! \param sender Module that sent topic
//! \param data Data for topic
//! \param target Target module (or broadcasted)
//!
void ModuleVehicleArdupilot::NewTopicData(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    if(this->m_TopicToControllers.find(topicName) == m_TopicToControllers.cend())
    {
        throw std::runtime_error("Attempting to send a topic that the vehicle module link has no knowledge of");
    }

    Controllers::IController<mavlink_message_t, int> *controller = m_TopicToControllers.at(topicName);

    if(controller->ContainsAction(Controllers::Actions::SEND) == false)
    {
        throw std::runtime_error("Attempting to send a topic to a controller that has no send action");
    }
    Controllers::IActionSend<MaceCore::TopicDatagram, int> *sendAction = dynamic_cast<Controllers::IActionSend<MaceCore::TopicDatagram, int>*>(controller);
    sendAction->Send(data, 255, this->GetAttachedMavlinkEntity());
}


//!
//! \brief New Spooled topic given
//!
//! Spooled topics are stored on the core's datafusion.
//! This method is used to notify other modules that there exists new data for the given components on the given module.
//! \param topicName Name of topic given
//! \param sender Module that sent topic
//! \param componentsUpdated Components in topic that where updated
//! \param target Target moudle (or broadcast)
//!
void ModuleVehicleArdupilot::NewTopicSpooled(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target)
{
    if(topicName == m_VehicleMissionTopic.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleMissionTopic.Name(), sender);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++){
            if(componentsUpdated.at(i) == MissionTopic::MissionItemTopic::Name()) {
                std::shared_ptr<MissionTopic::MissionItemTopic> component = std::make_shared<MissionTopic::MissionItemTopic>();
                m_VehicleMissionTopic.GetComponent(component, read_topicDatagram);
            }
            else if(componentsUpdated.at(i) == MissionTopic::MissionListTopic::Name()){
                std::shared_ptr<MissionTopic::MissionListTopic> component = std::make_shared<MissionTopic::MissionListTopic>();
                m_VehicleMissionTopic.GetComponent(component, read_topicDatagram);
            }
        }
    }
}


//!
//! \brief Cause the state machine to update it's states
//!
void ModuleVehicleArdupilot::ProgressStateMachineStates()
{
    m_Mutex_StateMachine.lock();
    stateMachine->ProcessStateTransitions();
    stateMachine->UpdateStates();
    m_Mutex_StateMachine.unlock();
}
