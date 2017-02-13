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
    //bool serverStarted = groundStation->StartTCPServer();
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
    if(m_TopicNotifier.find(topicName) == m_TopicNotifier.cend()) {
        m_TopicNotifier.insert({topicName, {}});
    }
    m_TopicNotifier[topicName].push_back(sender);
}

void MaceCore::NewTopicDataValues(const std::string &topicName, const int senderID, const TIME &time, const TopicDatagram &value) {

    std::vector<std::string> components = value.ListNonTerminals();

    m_DataFusion->setTopicDatagram(topicName, senderID, time, value);


    //list through all interested parties and notify of new topic data
    if(m_TopicNotifier.find(topicName) != m_TopicNotifier.cend())
    {
        for(auto it = m_TopicNotifier.at(topicName).cbegin() ; it != m_TopicNotifier.at(topicName).cend() ; ++it) {
            (*it)->NewTopic(topicName, senderID, components);
        }
    }
}


/////////////////////////////////////////////////////////////////////////
/// GENERAL MODULE EVENTS
/////////////////////////////////////////////////////////////////////////
void MaceCore::RequestVehicleArm(const void* sender, const MissionItem::ActionArm &arm)
{
    int vehicleID = arm.getVehicleID();
    m_VehicleIDToPort.at(vehicleID)->MarshalCommand(VehicleCommands::CHANGE_VEHICLE_ARM,arm);
}
void MaceCore::RequestVehicleMode(const void *sender, const MissionItem::ActionChangeMode &changeMode)
{
    int vehicleID = changeMode.getVehicleID();
    m_VehicleIDToPort.at(vehicleID)->MarshalCommand(VehicleCommands::CHANGE_VEHICLE_MODE,changeMode);
}

void MaceCore::RequestCurrentVehicleMission(const void *sender, const int &vehicleID)
{
    m_VehicleIDToPort.at(vehicleID)->MarshalCommand(VehicleCommands::REQUEST_CURRENT_MISSION_QUEUE,vehicleID);
}

void MaceCore::RequestVehicleHomePosition(const void* sender, const int &vehicleID)
{
    m_VehicleIDToPort.at(vehicleID)->MarshalCommand(VehicleCommands::REQUEST_VEHICLE_HOME,vehicleID);
}

void MaceCore::SetVehicleHomePosition(const void *sender, const MissionItem::SpatialHome &vehicleHome)
{
    std::cout<<"I have been told to set the home position"<<std::endl;
    m_VehicleIDToPort.at(vehicleHome.getVehicleID())->MarshalCommand(VehicleCommands::SET_VEHICLE_HOME,vehicleHome);
}

void MaceCore::RequestVehicleClearAutoMission(const void* sender, const int &vehicleID)
{
    m_VehicleIDToPort.at(vehicleID)->MarshalCommand(VehicleCommands::REQUEST_CLEAR_MISSION_QUEUE,vehicleID);
}

void MaceCore::RequestVehicleClearGuidedMission(const void* sender, const int &vehicleID)
{
    m_VehicleIDToPort.at(vehicleID)->MarshalCommand(VehicleCommands::REQUEST_CLEAR_GUIDED_QUEUE,vehicleID);
}

/////////////////////////////////////////////////////////////////////////
/// VEHICLE EVENTS
/////////////////////////////////////////////////////////////////////////
void MaceCore::NewConstructedVehicle(const void *sender, const int &newVehicleObserved)
{
    IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
    m_VehicleIDToPort.insert({newVehicleObserved,vehicle});
    m_DataFusion->AddAvailableVehicle(newVehicleObserved);

//    m_GroundStation->NewlyAvailableVehicle(newVehicleObserved);
    m_GroundStation->MarshalCommand(GroundStationCommands::NEW_AVAILABLE_VEHICLE,newVehicleObserved);
//    m_RTA->MarshalCommand(RTACommands::NEW_AVAILABLE_VEHICLE,newVehicleObserved);
//    m_PathPlanning->MarshalCommand(PathPlanningCommands::NEW_AVAILABLE_VEHICLE,newVehicleObserved);
}


/*
void MaceCore::NewVehicleMessage(const void *sender, const TIME &time, const VehicleMessage &vehicleMessage)
{
    IModuleCommandVehicle* vehicleModule = (IModuleCommandVehicle*)sender;
    int sendersID = 0;
    bool rtnValue = m_DataFusion->HandleVehicleMessage(vehicleMessage,sendersID);
    if(rtnValue == false && this->VehicleCheck(sendersID) == false){
        counter = counter + 1;
        vehicleModule->MarshalCommand(VehicleCommands::CREATE_VEHICLE_OBJECT, sendersID);
    }else{
        std::string tmpString = "NA";
        //m_PathPlanning->MarshalCommand(PathPlanningCommands::UPDATED_POSITION_DYNAMICS, tmpString);
        //m_GroundStation->MarshalCommand(GroundStationCommands::UPDATED_POSITION_DYNAMICS, tmpString);
    }
}
*/

//void MaceCore::NewPositionDynamics(const void* sender, const TIME &time, const Eigen::Vector3d &pos, const Eigen::Vector3d &vel)
//{
//    IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
//    std::string ID = m_VehiclePTRToID.at(vehicle);

//    m_DataFusion->AddPositionDynamics(ID, time, pos, vel);

//    m_PathPlanning->MarshalCommand(PathPlanningCommands::UPDATED_POSITION_DYNAMICS, ID);
//    //m_GroundStation->MarshalCommand(GroundStationCommands::UPDATED_POSITION_DYNAMICS, ID);
//}


//void MaceCore::NewDynamicsDynamics(const void* sender, const TIME &time, const Eigen::Vector3d &attitude, const Eigen::Vector3d &attitudeRate)
//{
//    IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
//    std::string ID = m_VehiclePTRToID.at(vehicle);

//    m_DataFusion->AddAttitudeDynamics(ID, time, attitude, attitudeRate);

//    m_PathPlanning->MarshalCommand(PathPlanningCommands::UPDATED_ATTITUDE_DYNAMICS, ID);
//    //m_GroundStation->MarshalCommand(GroundStationCommands::UPDATED_ATTITUDE_DYNAMICS, ID);
//}


//void MaceCore::NewVehicleLife(const void* sender, const TIME &time, const VehicleLife &life)
//{
//    IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
//    std::string ID = m_VehiclePTRToID.at(vehicle);

//    m_DataFusion->AddVehicleLife(ID, time, life);

//    m_PathPlanning->MarshalCommand(PathPlanningCommands::UPDATED_VEHICLE_LIFE, ID);
//    //m_GroundStation->MarshalCommand(GroundStationCommands::UPDATED_VEHICLE_LIFE, ID);
//}


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
    throw std::runtime_error("Not Implimented");
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
