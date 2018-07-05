#include "mace_core.h"

#include <stdexcept>
#include <iostream>

#include "module_characteristics.h"

namespace MaceCore
{

MaceCore::MaceCore() :
    m_GroundStation(NULL),
    m_PathPlanning(NULL),
    m_RTA(NULL)
{

}


/////////////////////////////////////////////////////////////////////////
/// CONFIGURE CORE
/////////////////////////////////////////////////////////////////////////


void MaceCore::AddDataFusion(const std::shared_ptr<MaceData> dataFusion)
{
    m_DataFusion = dataFusion;
}

void MaceCore::AddModule(const std::shared_ptr<ModuleBase> &module)
{
    std::vector<TopicCharacteristic> topics = module->GetEmittedTopics();
    for(auto it = topics.cbegin() ; it != topics.cend() ; ++it) {
        this->AddTopicCharacteristic(module.get(), *it);
    }
}

void MaceCore::AddVehicle(const std::string &ID, const std::shared_ptr<IModuleCommandVehicle> &vehicle)
{
    std::lock_guard<std::mutex> guard(m_VehicleMutex);

    if(m_VehicleIDToPtr.find(ID) != m_VehicleIDToPtr.cend())
        throw std::runtime_error("Vehicle ID already exists");

    m_VehicleIDToPtr.insert({ID, vehicle.get()});
    m_VehiclePTRToID.insert({vehicle.get(), ID});

    AddModule(vehicle);

    vehicle->addListener(this);
    vehicle->addTopicListener(this);


}


void MaceCore::RemoveVehicle(const std::string &ID)
{
    if(m_VehicleIDToPtr.find(ID) == m_VehicleIDToPtr.cend())
        throw std::runtime_error("Vehicle does not exists");

    m_VehiclePTRToID.erase(m_VehicleIDToPtr.at(ID));
    m_VehicleIDToPtr.erase(m_VehicleIDToPtr.find(ID));


    m_DataFusion->RemoveVehicle(ID);
}


//The following add the appropriate modules to the core
void MaceCore::AddExternalLink(const std::shared_ptr<IModuleCommandExternalLink> &externalLink)
{
    AddModule(externalLink);

    externalLink->addListener(this);
    externalLink->addTopicListener(this);
    m_ExternalLink.push_back(externalLink);

    //if there is a ground station, notify this new external link about the existance of GS
    if(m_GroundStation != NULL)
    {
        externalLink->MarshalCommand(ExternalLinkCommands::NEWLY_AVAILABLE_MODULE, m_GroundStation->GetCharacteristic());
    }

    //if there is an RTA module, notify this new external link about the existance of the rta module
    if(m_RTA != NULL)
    {
        externalLink->MarshalCommand(ExternalLinkCommands::NEWLY_AVAILABLE_MODULE, m_RTA->GetCharacteristic());
    }


}

void MaceCore::AddGroundStationModule(const std::shared_ptr<IModuleCommandGroundStation> &groundStation)
{
    groundStation->addListener(this);
    groundStation->addTopicListener(this);
    groundStation->StartTCPServer();
    m_GroundStation = groundStation;


    //notify all existing external links about new ground station.
    if(m_ExternalLink.size() > 0)
    {
        for (std::list<std::shared_ptr<IModuleCommandExternalLink>>::iterator it=m_ExternalLink.begin(); it!=m_ExternalLink.end(); ++it)
        {
            (*it)->MarshalCommand(ExternalLinkCommands::NEWLY_AVAILABLE_MODULE, m_GroundStation->GetCharacteristic());
        }
    }

    AddModule(groundStation);
}

void MaceCore::AddPathPlanningModule(const std::shared_ptr<IModuleCommandPathPlanning> &pathPlanning)
{
    pathPlanning->addListener(this);
    pathPlanning->addTopicListener(this);
    m_PathPlanning = pathPlanning;

    AddModule(pathPlanning);
}

void MaceCore::AddROSModule(const std::shared_ptr<IModuleCommandROS> &ros)
{
    ros->addListener(this);
    ros->addTopicListener(this);
    m_ROS = ros;

    AddModule(ros);
}

void MaceCore::AddRTAModule(const std::shared_ptr<IModuleCommandRTA> &rta)
{
    rta->addListener(this);
    rta->addTopicListener(this);
    m_RTA = rta;

    //notify all existing external links about new RTA module
    if(m_ExternalLink.size() > 0)
    {
        for (std::list<std::shared_ptr<IModuleCommandExternalLink>>::iterator it=m_ExternalLink.begin(); it!=m_ExternalLink.end(); ++it)
        {
            (*it)->MarshalCommand(ExternalLinkCommands::NEWLY_AVAILABLE_MODULE, m_RTA->GetCharacteristic());
        }
    }

    AddModule(rta);
}

void MaceCore::AddSensorsModule(const std::shared_ptr<IModuleCommandSensors> &sensors)
{
    sensors->addListener(this);
    sensors->addTopicListener(this);
    m_Sensors = sensors;

    AddModule(sensors);
}

//This ends the functions adding appropriate modules


void MaceCore::AddTopicCharacteristic(const ModuleBase *sender, const TopicCharacteristic &topic) {

    if(m_TopicsToReceive.find(sender) == m_TopicsToReceive.cend())
    {
        m_TopicsToReceive.insert({sender, {}});
    }

    if(m_TopicsToReceive.at(sender).find(topic.Name()) == m_TopicsToReceive.at(sender).cend())
    {
        m_TopicsToReceive.at(sender).insert({topic.Name(), topic});
    }
}

void MaceCore::Subscribe(ModuleBase* sender, const std::string &topicName, const std::vector<int> &senderIDs, const std::vector<std::string> &components)
{
    UNUSED(senderIDs);
    UNUSED(components);

    if(m_TopicNotifier.find(topicName) == m_TopicNotifier.cend()) {
        m_TopicNotifier.insert({topicName, {}});
    }
    m_TopicNotifier[topicName].push_back(sender);
}

void MaceCore::NewTopicDataValues(const ModuleBase* moduleFrom, const std::string &topicName, const ModuleCharacteristic &sender, const TIME &time, const TopicDatagram &value, const OptionalParameter<ModuleCharacteristic> &target)
{
    if(this->m_TopicsToReceive.at(moduleFrom).find(topicName) == m_TopicsToReceive.at(moduleFrom).cend())
    {
        throw std::runtime_error("Topic emitted to MaceCore was not expected");
    }

    if(this->m_TopicsToReceive.at(moduleFrom).at(topicName).Spooled() == true)
    {
        std::vector<std::string> components = value.ListNonTerminals();

        m_DataFusion->setTopicDatagram(topicName, sender.ID, time, value);

        //list through all interested parties and notify of new topic data
        if(m_TopicNotifier.find(topicName) != m_TopicNotifier.cend())
        {
            for(std::vector<ModuleBase*>::const_iterator it = m_TopicNotifier.at(topicName).cbegin() ; it != m_TopicNotifier.at(topicName).cend() ; ++it) {
                if((*it) == moduleFrom) continue;
                (*it)->NewTopicSpooled(topicName, sender, components, target);
            }
        }
    }
    else
    {
        //list through all interested parties and notify of new topic data
        if(m_TopicNotifier.find(topicName) != m_TopicNotifier.cend())
        {
            for(std::vector<ModuleBase*>::const_iterator it = m_TopicNotifier.at(topicName).cbegin() ; it != m_TopicNotifier.at(topicName).cend() ; ++it) {
                if((*it) == moduleFrom) continue;
                (*it)->NewTopicData(topicName, sender, value, target);
            }
        }
    }


}

void MaceCore::NewTopicDataValues(const ModuleBase* moduleFrom, const std::string &topicName, const int senderID, const TIME &time, const TopicDatagram &value) {

    //printf("Deprecated!\n");
    //throw std::runtime_error("Deprecated");

    std::vector<std::string> components = value.ListNonTerminals();

    m_DataFusion->setTopicDatagram(topicName, senderID, time, value);

    ModuleCharacteristic sender;
    sender.ID = senderID;
    sender.Class = ModuleClasses::VEHICLE_COMMS;

    //list through all interested parties and notify of new topic data
    if(m_TopicNotifier.find(topicName) != m_TopicNotifier.cend())
    {
        for(std::vector<ModuleBase*>::const_iterator it = m_TopicNotifier.at(topicName).cbegin() ; it != m_TopicNotifier.at(topicName).cend() ; ++it) {
            if((*it) == moduleFrom) continue;
            (*it)->NewTopicSpooled(topicName, sender, components);
        }
    }
}


/////////////////////////////////////////////////////////////////////////
/// GENERAL MODULE EVENTS
/////////////////////////////////////////////////////////////////////////

void MaceCore::RequestDummyFunction(const void *sender, const int &vehicleID)
{
    UNUSED(sender);
    //    UNUSED(vehicleID);

    //    try{
    //        std::cout<<"Saw a request dummy function"<<std::endl;
    //        m_VehicleIDToPort.at(vehicleID)->MarshalCommand(VehicleCommands::REQUEST_VEHICLE_HOME,vehicleID);
    //    }catch(const std::out_of_range &oor){

    //    }
    if(m_RTA) {
        m_RTA->MarshalCommand(RTACommands::TEST_FUNCTION, vehicleID);
    }
}

void MaceCore::Event_ForceVehicleDataSync(const ModuleBase *sender, const int &targetSystemID)
{
    MarshalCommandToVehicle<int>(targetSystemID, VehicleCommands::REQUEST_DATA_SYNC, ExternalLinkCommands::REQUEST_DATA_SYNC, targetSystemID, sender->GetCharacteristic());
}

void MaceCore::Event_IssueCommandSystemArm(const ModuleBase* sender, const CommandItem::ActionArm &command)
{
    UNUSED(sender);
    int vehicleID = command.getTargetSystem();
    MarshalCommandToVehicle<CommandItem::ActionArm>(vehicleID, VehicleCommands::CHANGE_VEHICLE_ARM, ExternalLinkCommands::CHANGE_VEHICLE_ARM, command, sender->GetCharacteristic());
}

void MaceCore::Event_IssueCommandTakeoff(const ModuleBase* sender, const CommandItem::SpatialTakeoff &command)
{
    UNUSED(sender);
    int vehicleID = command.getTargetSystem();
    MarshalCommandToVehicle<CommandItem::SpatialTakeoff>(vehicleID, VehicleCommands::REQUEST_VEHICLE_TAKEOFF, ExternalLinkCommands::REQUEST_VEHICLE_TAKEOFF, command, sender->GetCharacteristic());
}

void MaceCore::Event_IssueCommandLand(const ModuleBase* sender, const CommandItem::SpatialLand &command)
{
    UNUSED(sender);
    int vehicleID = command.getTargetSystem();
    MarshalCommandToVehicle<CommandItem::SpatialLand>(vehicleID, VehicleCommands::REQUEST_VEHICLE_LAND, ExternalLinkCommands::REQUEST_VEHICLE_LAND, command, sender->GetCharacteristic());
}

void MaceCore::Event_IssueCommandRTL(const ModuleBase* sender, const CommandItem::SpatialRTL &command)
{
    UNUSED(sender);
    int vehicleID = command.getTargetSystem();
    MarshalCommandToVehicle<CommandItem::SpatialRTL>(vehicleID, VehicleCommands::REQUEST_VEHICLE_RTL, ExternalLinkCommands::REQUEST_VEHICLE_RTL, command, sender->GetCharacteristic());
}

void MaceCore::Event_IssueMissionCommand(const ModuleBase* sender, const CommandItem::ActionMissionCommand &command)
{
    UNUSED(sender);
    int vehicleID = command.getTargetSystem();
    MarshalCommandToVehicle<CommandItem::ActionMissionCommand>(vehicleID, VehicleCommands::SET_MISSION_STATE, ExternalLinkCommands::SET_MISSION_STATE, command, sender->GetCharacteristic());
}

void MaceCore::Event_ChangeSystemMode(const ModuleBase *sender, const CommandItem::ActionChangeMode &command)
{
    UNUSED(sender);
    int vehicleID = command.getTargetSystem();
    MarshalCommandToVehicle<CommandItem::ActionChangeMode>(vehicleID, VehicleCommands::CHANGE_VEHICLE_MODE, ExternalLinkCommands::CHANGE_VEHICLE_MODE, command, sender->GetCharacteristic());
}

void MaceCore::Event_IssueGeneralCommand(const ModuleBase* sender, const std::shared_ptr<CommandItem::AbstractCommandItem> &command)
{
    switch(command->getCommandType())
    {
    case CommandItem::COMMANDITEM::CI_ACT_ARM:
    {
        return Event_IssueCommandSystemArm(sender, *((CommandItem::ActionArm*)(command.get())));
    }
    case CommandItem::COMMANDITEM::CI_NAV_TAKEOFF:
    {
        return Event_IssueCommandTakeoff(sender, *((CommandItem::SpatialTakeoff*)(command.get())));
    }
    case CommandItem::COMMANDITEM::CI_NAV_LAND:
    {
        return Event_IssueCommandLand(sender, *((CommandItem::SpatialLand*)(command.get())));
    }
    case CommandItem::COMMANDITEM::CI_NAV_RETURN_TO_LAUNCH:
    {
        return Event_IssueCommandRTL(sender, *((CommandItem::SpatialRTL*)(command.get())));
    }
    case CommandItem::COMMANDITEM::CI_ACT_MISSIONCOMMAND:
    {
        return Event_IssueMissionCommand(sender, *((CommandItem::ActionMissionCommand*)(command.get())));
    }
    case CommandItem::COMMANDITEM::CI_ACT_CHANGEMODE:
    {
        return Event_ChangeSystemMode(sender, *((CommandItem::ActionChangeMode*)(command.get())));
    }
    default:
    {
        std::cout << "General command not handled" << std::endl;
    }
    }
}

void MaceCore::Event_GetMission(const void *sender, const MissionItem::MissionKey &key)
{
    UNUSED(sender);
    int systemID = key.m_systemID;

    if(systemID == 0)
    {

    }else{
        try{
            MarshalCommandToVehicle<int>(systemID, VehicleCommands::REQUEST_MISSION, ExternalLinkCommands::REQUEST_MISSION, systemID);
        }catch(const std::out_of_range &oor){

        }
    }
}

void MaceCore::Event_GetOnboardMission(const void *sender, const int &systemID, const MissionItem::MISSIONTYPE &type)
{
    UNUSED(sender);

    VehicleCommands cmd = VehicleCommands::REQUEST_ONBOARD_AUTO_MISSION;
    ExternalLinkCommands cmd2 = ExternalLinkCommands::REQUEST_ONBOARD_AUTO_MISSION;

    if(type == MissionItem::MISSIONTYPE::AUTO)
    {
        //nothing to change since this is the default
    }else if(type == MissionItem::MISSIONTYPE::GUIDED){
        cmd = VehicleCommands::REQUEST_ONBOARD_GUIDED_MISSION;
        cmd2 = ExternalLinkCommands::REQUEST_ONBOARD_GUIDED_MISSION;
    }else{
        //we should throw some type of error
    }

    if(systemID == 0)
    {
        //how should we handle the case to transmit this to all vehicle instances
    }else{
        try{
            MarshalCommandToVehicle<int>(systemID, cmd, cmd2, systemID);
        }catch(const std::out_of_range &oor){

        }
    }
}

void MaceCore::Event_GetCurrentMission(const void *sender, const int &systemID)
{
    UNUSED(sender);
    MarshalCommandToVehicle<int>(systemID, VehicleCommands::REQUEST_CURRENT_MISSION, ExternalLinkCommands::REQUEST_CURRENT_MISSION, systemID);
}

void MaceCore::RequestClearVehicleMission(const void* sender, const Data::SystemDescription &systemID)
{
    UNUSED(sender);
    MarshalCommandToVehicle<Data::SystemDescription>(systemID.getSystemID(), VehicleCommands::CLEAR_CURRENT_MISSION, ExternalLinkCommands::CLEAR_CURRENT_MISSION, systemID.getSystemID());
}


void MaceCore::Event_GetHomePosition(const void* sender, const int &vehicleID)
{
    UNUSED(sender);
    MarshalCommandToVehicle<int>(vehicleID, VehicleCommands::REQUEST_VEHICLE_HOME, ExternalLinkCommands::REQUEST_VEHICLE_HOME, vehicleID);
}

void MaceCore::Event_SetHomePosition(const ModuleBase *sender, const CommandItem::SpatialHome &vehicleHome)
{
    UNUSED(sender);
    int vehicleID = vehicleHome.getTargetSystem();
    MarshalCommandToVehicle<CommandItem::SpatialHome>(vehicleID, VehicleCommands::SET_VEHICLE_HOME, ExternalLinkCommands::SET_VEHICLE_HOME, vehicleHome, sender->GetCharacteristic());
}

void MaceCore::RequestVehicleClearGuidedMission(const void* sender, const int &vehicleID)
{
    UNUSED(sender);

    MarshalCommandToVehicle<int>(vehicleID, VehicleCommands::CLEAR_ONBOARD_GUIDED_MISSION, ExternalLinkCommands::CLEAR_ONBOARD_GUIDED_MISSION, vehicleID);

}

void MaceCore::Event_SetGridSpacing(const void *sender, const double &gridSpacing)
{
    UNUSED(sender);
    m_DataFusion->UpdateGridSpacing(gridSpacing);
}


void MaceCore::Event_SetGlobalOrigin(const void *sender, const GeodeticPosition_3D &position)
{
    m_DataFusion->UpdateGlobalOrigin(position);

    if(m_PathPlanning && m_PathPlanning.get() != sender) {
        m_PathPlanning->MarshalCommand(PathPlanningCommands::NEWLY_UPDATED_GLOBAL_ORIGIN, position);
    }
    if(m_GroundStation && m_GroundStation.get() != sender) {
        m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_UPDATED_GLOBAL_ORIGIN, position);
    }
    if(m_RTA) {
        m_RTA->MarshalCommand(RTACommands::NEWLY_UPDATED_GLOBAL_ORIGIN, position);
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
/// SPECIFIC VEHICLE EVENTS: These events are associated from IModuleEventsVehicleVehicle
/////////////////////////////////////////////////////////////////////////////////////////

void MaceCore::EventVehicle_NewOnboardVehicleMission(const ModuleBase *sender, const MissionItem::MissionList &missionList)
{
    UNUSED(sender);
    //Update the core about the information
    MissionItem::MissionKey key = missionList.getMissionKey();
    m_DataFusion->receivedNewMission(missionList);
    bool isMissionCurrent = m_DataFusion->checkForCurrentMission(key);

    //Now update all potential listeners based on the type
    if(m_GroundStation)
    {
        if(m_DataFusion->getCurrentMissionValidity(missionList.getVehicleID()))
        {
            m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_CURRENT_MISSION,missionList.getMissionKey(), sender->GetCharacteristic());
        }
    }else if(m_ExternalLink.size() > 0)
    {
        for (std::list<std::shared_ptr<IModuleCommandExternalLink>>::iterator it=m_ExternalLink.begin(); it!=m_ExternalLink.end(); ++it)
        {
            (*it)->MarshalCommand(ExternalLinkCommands::NEWLY_AVAILABLE_ONBOARD_MISSION,missionList.getMissionKey(), sender->GetCharacteristic());
        }
    }
}

void MaceCore::EventVehicle_MissionACK(const void *sender, const MissionItem::MissionACK &ack)
{
    UNUSED(sender);

    //first we should update the core based on the acknowledgment information we had recieved
    //this will update the approriate keys as necessary
    MissionItem::MissionKey key = m_DataFusion->receivedMissionACKKey(ack.getMissionKey(), ack.getNewMissionState());

    if(m_ExternalLinkIDToPort.count(key.m_creatorID) > 0)
    {
        //this implies we can talk to the creator of this mission
        //let us send the acknowledgement to them
        m_ExternalLinkIDToPort.at(key.m_creatorID)->MarshalCommand(ExternalLinkCommands::RECEIVED_MISSION_ACK, ack);
    }
    //This may not be the place to do this
    bool isMissionCurrent = m_DataFusion->checkForCurrentMission(key);
    if((isMissionCurrent) && (m_GroundStation))
        m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_CURRENT_MISSION,key);
}

void MaceCore::EventVehicle_REJECTProposedMission(const void *sender, const MissionItem::MissionKey &key)
{
    UNUSED(sender);
    UNUSED(key);
}

void MaceCore::ExternalEvent_UpdateRemoteID(const void *sender, const int &remoteID)
{
    //KEN FIX THIS
    IModuleCommandExternalLink* externalLink = (IModuleCommandExternalLink*)sender;
    m_ExternalLinkIDToPort.insert({remoteID,externalLink});
}

void MaceCore::ExternalEvent_NewModule(const void *sender, const ModuleCharacteristic &newModule)
{
    std::lock_guard<std::mutex> guard(m_VehicleMutex);

    if(newModule.Class == ModuleClasses::VEHICLE_COMMS)
    {

        IModuleCommandExternalLink* externalLink = (IModuleCommandExternalLink*)sender;
        m_ExternalLinkIDToPort.insert({newModule.ID, externalLink});

        m_DataFusion->AddAvailableVehicle(newModule.ID, false);

        if(m_GroundStation)
            m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_VEHICLE, newModule.ID);

        if(m_RTA)
            m_RTA->MarshalCommand(RTACommands::NEWLY_AVAILABLE_VEHICLE, newModule.ID);

        if(m_ROS)
            m_ROS->MarshalCommand(ROSCommands::NEWLY_AVAILABLE_VEHICLE, newModule.ID);
    }
}

void MaceCore::EventVehicle_NewConstructedVehicle(const void *sender, const int &newVehicleObserved)
{
    std::lock_guard<std::mutex> guard(m_VehicleMutex);

    IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
    m_VehicleIDToPort.insert({newVehicleObserved,vehicle});
    m_DataFusion->AddAvailableVehicle(newVehicleObserved, true);


    if(m_RTA)
        m_RTA->MarshalCommand(RTACommands::NEWLY_AVAILABLE_VEHICLE, newVehicleObserved);

    if(m_PathPlanning)
        m_PathPlanning->MarshalCommand(PathPlanningCommands::NEWLY_AVAILABLE_VEHICLE, newVehicleObserved);

    if(m_ROS)
        m_ROS->MarshalCommand(ROSCommands::NEWLY_AVAILABLE_VEHICLE, newVehicleObserved);

    if(m_GroundStation.get() != NULL)
        m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_VEHICLE, newVehicleObserved);
    else if(m_ExternalLink.size() > 0)
    {
        ModuleCharacteristic module;
        module.ID = newVehicleObserved;
        module.Class = ModuleClasses::VEHICLE_COMMS;
        for (std::list<std::shared_ptr<IModuleCommandExternalLink>>::iterator it=m_ExternalLink.begin(); it!=m_ExternalLink.end(); ++it)
        {
            (*it)->MarshalCommand(ExternalLinkCommands::NEWLY_AVAILABLE_MODULE, module);
        }
    }
}

/////////////////////////////////////////////////////////////////////////
/// VEHICLE EVENTS
/////////////////////////////////////////////////////////////////////////

void MaceCore::GVEvents_NewHomePosition(const ModuleBase *sender, const CommandItem::SpatialHome &vehicleHome)
{
    UNUSED(sender);
    //TODO FIX KEN: We should incorporate a method that shall exist to understand who wants to receive
    //specific methods and information. Otherwise we may be blasting to an unknown world.
    //This is also bad as we are assuming that the only item calling this would be a vehicle instance
    m_DataFusion->UpdateVehicleHomePosition(vehicleHome);

    if(m_GroundStation && m_GroundStation.get() != sender)
        m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_HOME_POSITION,vehicleHome, sender->GetCharacteristic());
    else if(m_ExternalLink.size() > 0)
    {
        for (std::list<std::shared_ptr<IModuleCommandExternalLink>>::iterator it=m_ExternalLink.begin(); it!=m_ExternalLink.end(); ++it)
        {
            if(it->get() == sender)
            {
                continue;
            }
            (*it)->MarshalCommand(ExternalLinkCommands::NEWLY_AVAILABLE_HOME_POSITION,vehicleHome, sender->GetCharacteristic());
        }
    }

    //If targeting a vehicle on this module and not comming from self, set the home.
    if(m_VehicleIDToPort.find(vehicleHome.getTargetSystem()) != m_VehicleIDToPort.cend() && m_VehicleIDToPort.at(vehicleHome.getTargetSystem()) != sender)
    {
        m_VehicleIDToPort.at(vehicleHome.getTargetSystem())->MarshalCommand(VehicleCommands::SET_VEHICLE_HOME, vehicleHome, sender->GetCharacteristic());
    }
}

void MaceCore::GVEvents_MissionExeStateUpdated(const void *sender, const MissionItem::MissionKey &missionKey, const Data::MissionExecutionState &missionExeState)
{
    UNUSED(sender);
    //TODO FIX KEN: We should incorporate a method that shall exist to understand who wants to receive
    //specific methods and information. Otherwise we may be blasting to an unknown world.
    //This is also bad as we are assuming that the only item calling this would be a vehicle instance
    m_DataFusion->updateMissionExeState(missionKey,missionExeState);
    if(m_GroundStation)
        m_GroundStation->MarshalCommand(GroundStationCommands::NEW_MISSION_EXE_STATE,missionKey);
    else if(m_ExternalLink.size() > 0)
    {
        for (std::list<std::shared_ptr<IModuleCommandExternalLink>>::iterator it=m_ExternalLink.begin(); it!=m_ExternalLink.end(); ++it)
        {
            (*it)->MarshalCommand(ExternalLinkCommands::NEW_MISSION_EXE_STATE,missionKey);
        }
    }
}

void MaceCore::GVEvents_MissionItemAchieved(const void *sender, const MissionItem::MissionItemAchieved &achieved)
{
    UNUSED(sender);
    UNUSED(achieved);
    //I dont know if we need to do anything with this?
}

void MaceCore::GVEvents_MissionItemCurrent(const void *sender, const MissionItem::MissionItemCurrent &current)
{
    UNUSED(sender);
    m_DataFusion->updateCurrentMissionItem(current);
}

void MaceCore::ConfirmedOnboardVehicleMission(const void *sender, const MissionItem::MissionKey &missionKey)
{
    UNUSED(sender);
    UNUSED(missionKey);
    //m_DataFusion->updateOnboardMissions(missionKey);
}

void MaceCore::NewCurrentVehicleMission(const void *sender, const MissionItem::MissionKey &missionKey)
{
    UNUSED(sender);
    m_DataFusion->checkForCurrentMission(missionKey);

    if(m_GroundStation)
        m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_CURRENT_MISSION,missionKey);
}

/////////////////////////////////////////////////////////////////////////
/// EXTERNAL LINK EVENTS
/////////////////////////////////////////////////////////////////////////

void MaceCore::ExternalEvent_MissionACK(const void* sender, const MissionItem::MissionACK &missionACK)
{
    UNUSED(sender);
    std::cout<<"The core has seen an event from the external link confirming the mission"<<std::endl;
    //first we should update the core based on the acknowledgment information we had recieved
    //this will update the approriate keys as necessary
    MissionItem::MissionKey key = m_DataFusion->receivedMissionACKKey(missionACK.getMissionKey(), missionACK.getNewMissionState());

    //This may not be the place to do this
    bool isMissionCurrent = m_DataFusion->checkForCurrentMission(key);
    if((isMissionCurrent) && (m_GroundStation))
        m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_CURRENT_MISSION,key);
}

void MaceCore::ExternalEvent_NewOnboardMission(const ModuleBase *sender, const MissionItem::MissionKey &mission)
{
    //If we have an GS module, assume it is interested in downloading mission and request external link to download mission from aircraft
    if(m_GroundStation != NULL)
    {
        ModuleCharacteristic requestFrom;
        requestFrom.ID = m_GroundStation->GetID();
        requestFrom.Class = ModuleClasses::GROUND_STATION;

        if(sender->ModuleClass() == ModuleClasses::EXTERNAL_LINK)
        {
            ((IModuleCommandExternalLink*)sender)->MarshalCommand(ExternalLinkCommands::REQUEST_MISSION, mission, requestFrom);
        }
    }
}


//!
//! \brief MaceCore has been notified that a new boundary exists on a remote module.
//!
//!
//! \param sender Module that is communicating with remote module
//! \param data Data about boundary
//!
void MaceCore::ExternalEvent_NewBoundary(const ModuleBase *sender, const NewBoundaryData &data)
{
    std::cout << "A Remote boundary was detected" << std::endl;

    ///////////////////////
    ///MTB Logic goes here to decide if this mace instance is interested in the boundary recevied.
    ///    For now I am going to consult if it contains a vehicle attached to this instance.
    ///    Other things could be done here, the RTA or PP module can be consulted and asked if interested.
    ///////////////////////

    //! Set to true if this instance is interested in module
    bool interestedInBoundary = false;

    //! A module making request for the boundary.
    //! This is needed so the remote instance knows where to send the boundary.
    //! If multiple modules on this instance are interested in the boundary, only one of them needs to be set
    //!   Each module of interest will have a chance to access the downloaded boundary later.
    ModuleCharacteristic requestor;


    // pull list of all vehicles in the boundary
    std::vector<int> vehicles = data.Characteistic.List();

    //If global boundary and at least has a module attached then make interested
    if(vehicles.size() == 0)
    {
        if(m_RTA != nullptr)
        {
            requestor = m_RTA->GetCharacteristic();
            interestedInBoundary = true;
        }
        if(m_PathPlanning != nullptr)
        {
            requestor = m_PathPlanning->GetCharacteristic();
            interestedInBoundary = true;
        }
        if(m_VehicleIDToPort.size() > 0)
        {

            requestor = m_VehicleIDToPtr.cbegin()->second->GetCharacteristic();
            interestedInBoundary = true;
        }
    }

    //Check if an attached vehicle is part of vehicles in the boundary
    for(auto it = m_VehicleIDToPort.cbegin() ; it != m_VehicleIDToPort.cend() ; ++it)
    {
        int attachedVehicleID = it->first;
        if(std::find(vehicles.begin(), vehicles.end(), attachedVehicleID) != vehicles.end())
        {
            requestor = it->second->GetCharacteristic();
            interestedInBoundary = true;
        }
    }


    //if interested issue a download request
    if(interestedInBoundary == true)
    {
        if(sender->ModuleClass() == ModuleClasses::EXTERNAL_LINK)
        {
            ((IModuleCommandExternalLink*)sender)->MarshalCommand(ExternalLinkCommands::REQUEST_REMOTE_BOUNDARY, std::make_tuple(data.Sender, data.RemoteIdentifier), requestor);
        }
    }

    /*
    if(m_RTA != NULL)
    {
        //TEMPORARY
        //pull from key, this should probably be the key to the RTA module. i.e. m_RTA->GetCharacterisic()
        //In this case I know that the RTA module that is interested in the same computer with this vehicle.
        ModuleCharacteristic requestFrom;
//        requestFrom.ID = 1;
//        requestFrom.Class = ModuleClasses::VEHICLE_COMMS;
        requestFrom.ID = key.m_systemID;
        requestFrom.Class = ModuleClasses::RTA;

        if(sender->ModuleClass() == ModuleClasses::EXTERNAL_LINK)
        {
            ((IModuleCommandExternalLink*)sender)->MarshalCommand(ExternalLinkCommands::REQUEST_BOUNDARY, key, requestFrom);
        }
    }
    */
}

void MaceCore::ExternalEvent_RequestingDataSync(const void *sender, const int &targetID)
{
    std::unordered_map<std::string, TopicDatagram> topicMap = m_DataFusion->getAllLatestTopics(targetID);
    for(auto it = topicMap.cbegin() ; it != topicMap.cend() ; ++it) {
        std::vector<std::string> components = it->second.ListNonTerminals();
        ModuleBase* base = (ModuleBase*)sender;
        //base->NewTopic(it->first,targetID,components);
        throw std::runtime_error("Requesting Data Sync Not Implemented");
    }
}

void MaceCore::ExternalEvent_FinishedRXMissionList(const void *sender, const MissionItem::MissionList &missionList)
{
    UNUSED(sender);
    MissionItem::MISSIONSTATE state = missionList.getMissionTXState();
    MissionItem::MissionKey key = missionList.getMissionKey();

    m_DataFusion->receivedNewMission(missionList);
    bool isMissionCurrent = m_DataFusion->checkForCurrentMission(key);

    //This may not be the place to do this
    if((isMissionCurrent) && (m_GroundStation))
        m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_CURRENT_MISSION,key);
    else if(state ==  MissionItem::MISSIONSTATE::RECEIVED)//This implies that the mission state has just moved from proposed to received
    {
        int vehicleID = missionList.getVehicleID();
        MarshalCommandToVehicle<MissionItem::MissionList>(vehicleID, VehicleCommands::UPLOAD_MISSION, ExternalLinkCommands::UPLOAD_MISSION, missionList);
    }
}

void MaceCore::ExternalEvent_FinishedRXBoundaryList(const void *sender, const BoundaryItem::BoundaryList &boundaryList)
{
    UNUSED(sender);

    throw std::runtime_error("External Link Finished Receiving boundary list not implimented");

    /*

    BoundaryItem::BOUNDARYTYPE type = boundaryList.getBoundaryType();
    BoundaryItem::BoundaryKey key = boundaryList.getBoundaryKey();

    std::cout << "External event finished RX boundary list" << std::endl;

//    m_DataFusion->receivedNewBoundaryList(boundaryList);

    if(m_PathPlanning) {
        // Marshal command for new boundary list
        if(type == BoundaryItem::BOUNDARYTYPE::OPERATIONAL_FENCE) {
            m_PathPlanning->MarshalCommand(PathPlanningCommands::NEWLY_UPDATED_OPERATIONAL_FENCE, boundaryList);
        }
    }

    if(m_RTA) {
        // Marshal command for new boundary list
        if(type == BoundaryItem::BOUNDARYTYPE::RESOURCE_FENCE) {
            m_RTA->MarshalCommand(RTACommands::NEWLY_UPDATED_RESOURCE_FENCE, boundaryList);
        }
        if(type == BoundaryItem::BOUNDARYTYPE::OPERATIONAL_FENCE) {
            m_RTA->MarshalCommand(RTACommands::NEWLY_UPDATED_OPERATIONAL_FENCE, boundaryList);
        }
    }
    */
}



// TODO: Pat/Ken - Event_SetVehicleTargets or whatever

/////////////////////////////////////////////////////////////////////////
/// GROUND STATION EVENTS
/////////////////////////////////////////////////////////////////////////


//!
//! \brief Event fired when a new list of targets are produced for a specific vehicle
//! \param vehicleID Vechile new targets are to be applied to
//! \param target List of positional targets
//!
void MaceCore::GroundStationEvent()
{
}

void MaceCore::CommandNewVehicleMode(const std::string &vehicleMode)
{
    UNUSED(vehicleMode);
}

//!
//! \brief Event_UploadMission method calls the appropriate operation methods to migrate the proposed
//! mission list to the appropriate vehicle module for handling.
//! \param sender
//! \param missionList
//!
void MaceCore::GSEvent_UploadMission(const void *sender, const MissionItem::MissionList &missionList)
{
    UNUSED(sender);

    MissionItem::MissionList::MissionListStatus status = missionList.getMissionListStatus();

    if(status.state == MissionItem::MissionList::INCOMPLETE) //this checks to make sure the list is fully populated
        return;

    int vehicleID = missionList.getVehicleID();
    if(vehicleID == 0) //transmit this mission to all vehicles
    {
        for (std::map<int, IModuleCommandVehicle*>::iterator it=m_VehicleIDToPort.begin(); it!=m_VehicleIDToPort.end(); ++it){
            int nextSystemID = it->first;
            MissionItem::MissionKey key = m_DataFusion->appendAssociatedMissionMap(nextSystemID,missionList);
            MissionItem::MissionList correctedMission = missionList;
            correctedMission.setMissionKey(key);
            MarshalCommandToVehicle<MissionItem::MissionList>(vehicleID, VehicleCommands::UPLOAD_MISSION, ExternalLinkCommands::UPLOAD_MISSION, correctedMission);
        }
    }else{ //transmit the mission to a specific vehicle
        try{
            MissionItem::MissionKey key = m_DataFusion->appendAssociatedMissionMap(missionList);
            MissionItem::MissionList correctedMission = missionList;
            correctedMission.setMissionKey(key);
            MarshalCommandToVehicle<MissionItem::MissionList>(vehicleID, VehicleCommands::UPLOAD_MISSION, ExternalLinkCommands::UPLOAD_MISSION, correctedMission);

        }catch(const std::out_of_range &oor){

        }
    }
}

/////////////////////////////////////////////////////////////////////////
/// PATH PLANNING EVENTS
/////////////////////////////////////////////////////////////////////////

void MaceCore::EventPP_LoadOccupancyEnvironment(const ModuleBase *sender, const string &filePath)
{
    if(m_DataFusion->loadOccupancyEnvironment(filePath))
    {
        //we have loaded a new map which means we need to notify everyone

        //we dont have to check if PP exists here because we know it has to as it is the caller
        m_PathPlanning->MarshalCommand(PathPlanningCommands::NEWLY_LOADED_OCCUPANCY_MAP);
    }
}

void MaceCore::EventPP_LoadOctomapProperties(const ModuleBase *sender, const maps::OctomapSensorDefinition &properties)
{
    if(m_DataFusion->updateOctomapProperties(properties))
    {
        //we have loaded a new map which means we need to notify everyone

        //we dont have to check if PP exists here because we know it has to as it is the caller
        m_PathPlanning->MarshalCommand(PathPlanningCommands::NEWLY_LOADED_OCCUPANCY_MAP);
    }
}

void MaceCore::EventPP_LoadMappingProjectionProperties(const ModuleBase *sender, const maps::Octomap2DProjectionDefinition &properties)
{

}

//!
//! \brief Function to fire when a new boundary of some kind was generated by a module
//! \param sender Module that generated the boundary
//! \param key Key indicating the characteristics of the boundary
//! \param boundary Data for the boundary
//!
void MaceCore::Event_SetBoundary(const ModuleBase *sender, const BoundaryItem::BoundaryCharacterisic &characterstic, const BoundaryItem::BoundaryList &boundary)
{
    std::string list_str = "";
    if(characterstic.List().size() == 0)
    {
        list_str = "global";
    }
    std::vector<int> list = characterstic.List();
    for(auto it = list.cbegin() ; it != list.cend() ; ++it)
    {
        list_str += std::to_string(*it) + " ";
    }
    printf("Mace Core: Received a new Boundary\n  verticies: %d\n  Type: %s\n  Vehicles: %s\n", boundary.getQueueSize(), BoundaryItem::BoundaryTypeToString(characterstic.Type()).c_str(), list_str.c_str());

    //Update the underalying data object
    uint8_t key = m_DataFusion->setBoundaryByKey(characterstic, boundary);

    if(m_GroundStation.get() == sender)
    {
        printf("!!!!!!MADISON TESTING!!!!! - RTA module isn't full developed, so restricting transmission of boundary only when boundary came from GS\n");
        if(m_RTA && sender != m_RTA.get()) {
            m_RTA->MarshalCommand(RTACommands::NEWLY_AVAILABLE_BOUNDARY, key);
        }
    }

    // TODO-@Ken: Does the PP module need the operational fence?
//    if(m_PathPlanning && sender != m_PathPlanning) {
//        m_PathPlanning->MarshalCommand(PathPlanningCommands::NEWLY_UPDATED_OPERATIONAL_FENCE, boundary);
//    }

    if(m_ROS && sender != m_ROS.get())
    {
        m_ROS->MarshalCommand(ROSCommands::NEWLY_AVAILABLE_BOUNDARY, key);
    }

    if(m_GroundStation && sender != m_GroundStation.get()) {
        m_GroundStation->MarshalCommand(GroundStationCommands::NEWLY_AVAILABLE_BOUNDARY, key);
    }

    if(m_ExternalLink.size() > 0)
    {
        for (std::list<std::shared_ptr<IModuleCommandExternalLink>>::iterator it = m_ExternalLink.begin(); it != m_ExternalLink.end(); ++it)
        {
            if(it->get() == sender)
            {
                continue;
            }

            (*it)->MarshalCommand(ExternalLinkCommands::NEWLY_AVAILABLE_BOUNDARY, key, sender->GetCharacteristic());
        }
    }
}


//!
//! \brief Event fired to indicate what planning horizon is being utilized by the path planning module
//! \param horizon ID of the horizon being utilized
//!
void MaceCore::PlanningHorizon(const std::string &horizon)
{
    UNUSED(horizon);
    throw std::runtime_error("Not Implemented");
}

void MaceCore::ReplaceVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands)
{
    m_DataFusion->setVehicleDynamicsCommands(vehicleID, movementCommands);

    m_VehicleIDToPtr.at(vehicleID)->MarshalCommand(VehicleCommands::FOLLOW_NEW_COMMANDS);
}

void MaceCore::ReplaceAfterCurrentVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands)
{
    m_DataFusion->setVehicleDynamicsCommands(vehicleID, movementCommands);

    m_VehicleIDToPtr.at(vehicleID)->MarshalCommand(VehicleCommands::FINISH_AND_FOLLOW_COMMANDS);
}

void MaceCore::AppendVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands)
{
    std::vector<FullVehicleDynamics> commands = m_DataFusion->getVehicleDynamicsCommands(vehicleID);
    commands.insert(commands.end(), movementCommands.begin(), movementCommands.end());
    m_DataFusion->setVehicleDynamicsCommands(vehicleID, commands);

    m_VehicleIDToPtr.at(vehicleID)->MarshalCommand(VehicleCommands::COMMANDS_APPENDED);
}

void MaceCore::EventPP_New2DOccupancyMap(const void* sender, const mace::maps::Data2DGrid<mace::maps::OccupiedResult> &map)
{
    if(m_ROS)
        m_ROS->MarshalCommand(ROSCommands::NEWLY_COMPRESSED_OCCUPANCY_MAP, map);
}

void MaceCore::EventPP_NewDynamicMissionQueue(const ModuleBase *sender, const TargetItem::DynamicMissionQueue &queue)
{
    UNUSED(sender);

    //int vehicleID = queue.missionKey.m_systemID;
    //m_VehicleIDToPtr.at(vehicleID)->MarshalCommand(VehicleCommands::UPDATED_DYNAMIC_MISSION_QUEUE, queue);
}

void MaceCore::EventPP_NewPathFound(const void* sender, const std::vector<mace::state_space::StatePtr> &path)
{
    UNUSED(sender);
    if(m_ROS)
        m_ROS->MarshalCommand(ROSCommands::NEWLY_FOUND_PATH, path);
}

//!
//! \brief Event fired when a new occupancy map to be invoked when PathPlanning module generates a new occupancy map.
//! \param occupancyMap New occupancy map
//!
void MaceCore::NewOccupancyMap(const Eigen::MatrixXd &occupancyMap)
{
    m_DataFusion->OccupancyMap_ReplaceMatrix(occupancyMap);
}


//!
//! \brief Event fired when the PathPlanning modules determines that a set of cells should be modified on the occupancy map.
//!
//! This event may be faster than NewOccupancyMap when the matrix is large and the modifcations are sparse
//! \param commands List of cells to modify
//!
void MaceCore::ReplaceOccupancyMapCells(const std::vector<MatrixCellData<double>> &commands)
{

    std::function<void(Eigen::MatrixXd &)> func = [&commands](Eigen::MatrixXd &mat){
        ReplaceCellsInMatrix(mat, commands);
    };

    m_DataFusion->OccupancyMap_GenericOperation(func);

}

/////////////////////////////////////////////////////////////////////////
/// SENSOR MODULE EVENTS
/////////////////////////////////////////////////////////////////////////

void MaceCore::ROS_NewLaserScan(const octomap::Pointcloud &obj, const mace::pose::Position<mace::pose::CartesianPosition_3D> &position)
{
    octomap::Pointcloud copyObj = obj;
    m_DataFusion->insertGlobalObservation(copyObj, position);
    if(m_ROS)
        m_ROS->MarshalCommand(ROSCommands::NEWLY_UPDATED_3D_OCCUPANCY_MAP, 0); // TODO: Parse for vehicle ID
}

void MaceCore::ROS_NewLaserScan(const octomap::Pointcloud &obj, const mace::pose::Position<mace::pose::CartesianPosition_3D> &position, const mace::pose::Orientation_3D &orientation)
{
    octomap::Pointcloud copyObj = obj;
    m_DataFusion->insertObservation(copyObj, position, orientation);
    //Marshal Command To PP and RTA

    /*    ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
    }); *///this is a general publication event, however, no one knows explicitly how to handle

    /*    ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
        ptr->EventVehicle_NewConstructedVehicle(this, systemID);
    });*/ //this one explicitly calls mace_core and its up to you to handle in core

    //    if(m_PathPlanning)
    //        m_PathPlanning->MarshalCommand(PathPlanningCommands::NEWLY_UPDATED_OCCUPANCY_MAP, 0); // TODO: Parse for vehicle ID

    if(m_ROS)
        m_ROS->MarshalCommand(ROSCommands::NEWLY_UPDATED_3D_OCCUPANCY_MAP, 0); // TODO: Parse for vehicle ID

}

/////////////////////////////////////////////////////////////////////////
/// MACE COMMS EVENTS
/////////////////////////////////////////////////////////////////////////


} //END MaceCore Namespace
