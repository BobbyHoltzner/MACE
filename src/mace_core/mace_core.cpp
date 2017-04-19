#include "mace_core.h"

#include <stdexcept>
#include <iostream>

namespace MaceCore
{

MaceCore::MaceCore()
{

}


/////////////////////////////////////////////////////////////////////////
/// CONFIGURE CORE
/////////////////////////////////////////////////////////////////////////


void MaceCore::AddDataFusion(const std::shared_ptr<MaceData> dataFusion)
{
    m_DataFusion = dataFusion;
}

void MaceCore::AddVehicle(const std::string &ID, const std::shared_ptr<IModuleCommandVehicle> &vehicle)
{
    std::lock_guard<std::mutex> guard(m_VehicleMutex);

    if(m_VehicleIDToPtr.find(ID) != m_VehicleIDToPtr.cend())
        throw std::runtime_error("Vehicle ID already exists");

    m_VehicleIDToPtr.insert({ID, vehicle.get()});
    m_VehiclePTRToID.insert({vehicle.get(), ID});

    //m_DataFusion->AddVehicle(ID);

    vehicle->addListener(this);
    vehicle->addTopicListener(this);

    std::unordered_map<std::string, TopicStructure> topics = vehicle->GetTopics();
    for(auto it = topics.cbegin() ; it != topics.cend() ; ++it) {
        this->AddTopic(it->first, it->second);
    }
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
    externalLink->addListener(this);
    externalLink->addTopicListener(this);
}

void MaceCore::AddGroundStationModule(const std::shared_ptr<IModuleCommandGroundStation> &groundStation)
{
    groundStation->addListener(this);
    groundStation->addTopicListener(this);
    bool serverStarted = groundStation->StartTCPServer();
    UNUSED(serverStarted);
    m_GroundStation = groundStation;
}

void MaceCore::AddPathPlanningModule(const std::shared_ptr<IModuleCommandPathPlanning> &pathPlanning)
{
    pathPlanning->addListener(this);
    pathPlanning->addTopicListener(this);
    m_PathPlanning = pathPlanning;
}

void MaceCore::AddRTAModule(const std::shared_ptr<IModuleCommandRTA> &rta)
{
    rta->addListener(this);
    rta->addTopicListener(this);
    m_RTA = rta;
}

void MaceCore::AddSensorsModule(const std::shared_ptr<IModuleCommandSensors> &sensors)
{
    sensors->addListener(this);
    sensors->addTopicListener(this);
    m_Sensors = sensors;
}

//This ends the functions adding appropriate modules


void MaceCore::AddTopic(const std::string &topicName, const TopicStructure &topic) {
    m_Topics.insert({topicName, topic});
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

void MaceCore::NewTopicDataValues(const ModuleBase* moduleFrom, const std::string &topicName, const int senderID, const TIME &time, const TopicDatagram &value) {

    std::vector<std::string> components = value.ListNonTerminals();

    m_DataFusion->setTopicDatagram(topicName, senderID, time, value);

    //list through all interested parties and notify of new topic data
    if(m_TopicNotifier.find(topicName) != m_TopicNotifier.cend())
    {        
        for(auto it = m_TopicNotifier.at(topicName).cbegin() ; it != m_TopicNotifier.at(topicName).cend() ; ++it) {
            if((*it) == moduleFrom) continue;
            (*it)->NewTopic(topicName, senderID, components);
        }
    }
}


/////////////////////////////////////////////////////////////////////////
/// GENERAL MODULE EVENTS
/////////////////////////////////////////////////////////////////////////

void MaceCore::RequestDummyFunction(const void *sender, const int &vehicleID)
{
    if(vehicleID == 0)
    {
        for (std::map<int, IModuleCommandVehicle*>::iterator it=m_VehicleIDToPort.begin(); it!=m_VehicleIDToPort.end(); ++it){
            it->second->MarshalCommand(VehicleCommands::REQUEST_CURRENT_MISSION_QUEUE,vehicleID);
        }
    }else{
        try{
            m_VehicleIDToPort.at(vehicleID)->MarshalCommand(VehicleCommands::REQUEST_DUMMY_FUNCTION,vehicleID);
        }catch(const std::out_of_range &oor){

        }
    }
}

void MaceCore::RequestVehicleArm(const void* sender, const MissionItem::ActionArm &arm)
{
    UNUSED(sender);
    int vehicleID = arm.getVehicleID();
    if(vehicleID == 0)
    {
        for (std::map<int, IModuleCommandVehicle*>::iterator it=m_VehicleIDToPort.begin(); it!=m_VehicleIDToPort.end(); ++it){
            MissionItem::ActionArm newArm = arm;
            newArm.setVehicleID(it->first);
            it->second->MarshalCommand(VehicleCommands::CHANGE_VEHICLE_ARM,newArm);
        }
    }else{
        try{
            m_VehicleIDToPort.at(vehicleID)->MarshalCommand(VehicleCommands::CHANGE_VEHICLE_ARM,arm);
        }catch(const std::out_of_range &oor){

        }
    }

}
void MaceCore::RequestVehicleMode(const void *sender, const MissionItem::ActionChangeMode &changeMode)
{
    UNUSED(sender);
    int vehicleID = changeMode.getVehicleID();
    if(vehicleID == 0)
    {
        for (std::map<int, IModuleCommandVehicle*>::iterator it=m_VehicleIDToPort.begin(); it!=m_VehicleIDToPort.end(); ++it){
            MissionItem::ActionChangeMode newMode = changeMode;
            newMode.setVehicleID(it->first);
            it->second->MarshalCommand(VehicleCommands::CHANGE_VEHICLE_MODE,newMode);
        }
    }else{
        try{
            m_VehicleIDToPort.at(vehicleID)->MarshalCommand(VehicleCommands::CHANGE_VEHICLE_MODE,changeMode);
        }catch(const std::out_of_range &oor){

        }
    }
}

void MaceCore::RequestVehicleTakeoff(const void* sender, const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &vehicleTakeoff)
{
    UNUSED(sender);
    int vehicleID = vehicleTakeoff.getVehicleID();
    if(vehicleID == 0)
    {
        for (std::map<int, IModuleCommandVehicle*>::iterator it=m_VehicleIDToPort.begin(); it!=m_VehicleIDToPort.end(); ++it){
            MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> newTakeoff = vehicleTakeoff;
            newTakeoff.setVehicleID(it->first);
            it->second->MarshalCommand(VehicleCommands::REQUEST_VEHICLE_TAKEOFF,newTakeoff);
        }
    }else{
        try{
            m_VehicleIDToPort.at(vehicleID)->MarshalCommand(VehicleCommands::REQUEST_VEHICLE_TAKEOFF,vehicleTakeoff);
        }catch(const std::out_of_range &oor){

        }
    }
}


//!
//! \brief RequestSetVehicleMission method calls the appropriate handling operations to migrate the proposed
//! mission list to the appropriate vehicle module for handling.
//! \param sender
//! \param missionList
//!
void MaceCore::RequestSetVehicleMission(const void *sender, const MissionItem::MissionList &missionList)
{
    MissionItem::MissionList::MissionListStatus status = missionList.getMissionListStatus();

    if(status.state == MissionItem::MissionList::INCOMPLETE) //this checks to make sure the list is fully populated
        return;

    int vehicleID = missionList.getVehicleID();
    if(vehicleID == 0) //transmit this mission to all vehicles
    {
        for (std::map<int, IModuleCommandVehicle*>::iterator it=m_VehicleIDToPort.begin(); it!=m_VehicleIDToPort.end(); ++it){
            int nextSystemID = it->first;
            MissionItem::MissionList correctedMissionList = m_DataFusion->appenedAssociatedMissionMap(nextSystemID,missionList);
            if(it->second != sender){
                it->second->MarshalCommand(VehicleCommands::SET_CURRENT_MISSION_QUEUE,correctedMissionList);
            }
        }
    }else{ //transmit the mission to a specific vehicle
        try{
            MissionItem::MissionList correctedMissionList = m_DataFusion->appenedAssociatedMissionMap(missionList);
            IModuleCommandVehicle* module = m_VehicleIDToPort.at(vehicleID);
            if(module != sender){
                module->MarshalCommand(VehicleCommands::SET_CURRENT_MISSION_QUEUE,correctedMissionList);
            }

        }catch(const std::out_of_range &oor){

        }
    }
}
void MaceCore::RequestVehicleMission(const void *sender, const int &systemID)
{
    UNUSED(sender);
    if(systemID == 0)
    {
        for (std::map<int, IModuleCommandVehicle*>::iterator it=m_VehicleIDToPort.begin(); it!=m_VehicleIDToPort.end(); ++it){
            it->second->MarshalCommand(VehicleCommands::REQUEST_CURRENT_MISSION_QUEUE,systemID);
        }
    }else{
        try{
            m_VehicleIDToPort.at(systemID)->MarshalCommand(VehicleCommands::REQUEST_CURRENT_MISSION_QUEUE,systemID);
        }catch(const std::out_of_range &oor){

        }
    }
}
void MaceCore::RequestClearVehicleMission(const void* sender, const Data::SystemDescription &systemID)
{
    UNUSED(sender);
    if(systemID.getSystemID() == 0)
    {
        for (std::map<int, IModuleCommandVehicle*>::iterator it=m_VehicleIDToPort.begin(); it!=m_VehicleIDToPort.end(); ++it){
            it->second->MarshalCommand(VehicleCommands::REQUEST_CLEAR_MISSION_QUEUE,systemID);
        }
    }else{
        try{
            m_VehicleIDToPort.at(systemID.getSystemID())->MarshalCommand(VehicleCommands::REQUEST_CLEAR_MISSION_QUEUE,systemID);
        }catch(const std::out_of_range &oor){

        }
    }
}


void MaceCore::RequestVehicleHomePosition(const void* sender, const int &vehicleID)
{
    UNUSED(sender);
    if(vehicleID == 0)
    {
        for (std::map<int, IModuleCommandVehicle*>::iterator it=m_VehicleIDToPort.begin(); it!=m_VehicleIDToPort.end(); ++it){
            it->second->MarshalCommand(VehicleCommands::REQUEST_VEHICLE_HOME,vehicleID);
        }
    }else{
        try{
            m_VehicleIDToPort.at(vehicleID)->MarshalCommand(VehicleCommands::REQUEST_VEHICLE_HOME,vehicleID);
        }catch(const std::out_of_range &oor){

        }
    }
}

void MaceCore::SetVehicleHomePosition(const void *sender, const MissionItem::SpatialHome &vehicleHome)
{
    UNUSED(sender);
    int vehicleID = vehicleHome.getVehicleID();
    if(vehicleID == 0)
    {
        for (std::map<int, IModuleCommandVehicle*>::iterator it=m_VehicleIDToPort.begin(); it!=m_VehicleIDToPort.end(); ++it){
            MissionItem::SpatialHome newHome = vehicleHome;
            newHome.setVehicleID(it->first);
            it->second->MarshalCommand(VehicleCommands::SET_VEHICLE_HOME,newHome);
        }
    }else{
        try{
            m_VehicleIDToPort.at(vehicleID)->MarshalCommand(VehicleCommands::SET_VEHICLE_HOME,vehicleHome);
        }catch(const std::out_of_range &oor){

        }
    }
}

void MaceCore::RequestVehicleClearGuidedMission(const void* sender, const int &vehicleID)
{
    UNUSED(sender);
    if(vehicleID == 0)
    {
        for (std::map<int, IModuleCommandVehicle*>::iterator it=m_VehicleIDToPort.begin(); it!=m_VehicleIDToPort.end(); ++it){
            it->second->MarshalCommand(VehicleCommands::REQUEST_CLEAR_GUIDED_QUEUE,vehicleID);
        }
    }else{
        try{
            m_VehicleIDToPort.at(vehicleID)->MarshalCommand(VehicleCommands::REQUEST_CLEAR_GUIDED_QUEUE,vehicleID);
        }catch(const std::out_of_range &oor){

        }
    }
}

void MaceCore::UpdateGlobalOriginPosition(const void *sender, const MissionItem::SpatialHome &globalHome)
{
    UNUSED(sender);
    m_DataFusion->UpdateGlobalOrigin(globalHome);
}
/////////////////////////////////////////////////////////////////////////
/// VEHICLE EVENTS
/////////////////////////////////////////////////////////////////////////
void MaceCore::NewConstructedVehicle(const void *sender, const int &newVehicleObserved)
{
    std::lock_guard<std::mutex> guard(m_VehicleMutex);

    IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
    m_VehicleIDToPort.insert({newVehicleObserved,vehicle});
    m_DataFusion->AddAvailableVehicle(newVehicleObserved);

    if(m_GroundStation)
        m_GroundStation->MarshalCommand(GroundStationCommands::NEW_AVAILABLE_VEHICLE,newVehicleObserved);
    MissionItem::SpatialHome newOrigin;
    newOrigin.position = DataState::StateGlobalPosition(37.890810,-76.814833,0.0);
    m_DataFusion->UpdateGlobalOrigin(newOrigin);
}

void MaceCore::NewVehicleHomePosition(const void *sender, const MissionItem::SpatialHome &vehicleHome)
{
    UNUSED(sender);
    //TODO FIX KEN: We should incorporate a method that shall exist to understand who wants to receive
    //specific methods and information. Otherwise we may be blasting to an unknown world.
    //This is also bad as we are assuming that the only item calling this would be a vehicle instance
    m_DataFusion->UpdateVehicleHomePosition(vehicleHome);
}

void MaceCore::NewOnboardVehicleMission(const void *sender, const MissionItem::MissionList &missionList)
{
    MissionItem::MissionList correctedMissionList = m_DataFusion->appenedAssociatedMissionMap(missionList);
    m_DataFusion->updateOnboardMissions(correctedMissionList.getMissionKey());
    if(correctedMissionList.getMissionKey() != missionList.getMissionKey())
    {
        //this means that this instance of mace had already planned more items for that vehicle or had knowledge of more
        //missions available for that vehicle
        Data::MissionKeyChange keyChange(missionList.getMissionKey(),correctedMissionList.getMissionKey());
        IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
        vehicle->MarshalCommand(VehicleCommands::UPDATE_MISSION_KEY,keyChange);
    }
}

void MaceCore::ConfirmedOnboardVehicleMission(const void *sender, const Data::MissionKey &missionKey)
{
    m_DataFusion->updateOnboardMissions(missionKey);
}

void MaceCore::NewCurrentVehicleMission(const void *sender, const Data::MissionKey &missionKey)
{
    m_DataFusion->updateCurrentMission(missionKey);

    if(m_GroundStation)
        m_GroundStation->MarshalCommand(GroundStationCommands::NEW_AVAILABLE_CURRENT_MISSION,missionKey);
}

/////////////////////////////////////////////////////////////////////////
/// EXTERNAL LINK EVENTS
/////////////////////////////////////////////////////////////////////////

void MaceCore::TransferMissionToVehicle(const void *sender, const MissionItem::MissionList &missionList)
{
    RequestSetVehicleMission(sender,missionList);
}


/////////////////////////////////////////////////////////////////////////
/// RTA EVENTS
/////////////////////////////////////////////////////////////////////////


//!
//! \brief Event fired when a new list of targets are produced for a specific vehicle
//! \param vehicleID Vechile new targets are to be applied to
//! \param target List of positional targets
//!
void MaceCore::NewVehicleTargets(const std::string &vehicleID, const std::vector<Eigen::Vector3d> &target)
{
    m_DataFusion->setVehicleTarget(vehicleID, target);

    //m_PathPlanning->NewVehicleTarget(vehicleID);
}

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


/////////////////////////////////////////////////////////////////////////
/// PATH PLANNING EVENTS
/////////////////////////////////////////////////////////////////////////

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
/// MACE COMMS EVENTS
/////////////////////////////////////////////////////////////////////////


} //END MaceCore Namespace
