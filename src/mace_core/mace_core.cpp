#include "mace_core.h"

#include <stdexcept>
#include <iostream>

namespace MaceCore
{

MaceCore::MaceCore()
{
    insertFlag = false;
    counter = 0;
    counter_new_vehicle = 0;
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

    m_DataFusion->AddVehicle(ID);

    vehicle->addListener(this);

    if(m_RTA != NULL)
        m_RTA->MarshalCommand(RTACommands::NEW_VEHICLE, ID);

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

    if(m_RTA != NULL)
        m_RTA->MarshalCommand(RTACommands::REMOVE_VEHICLE, ID);
}


void MaceCore::AddRTAModule(const std::shared_ptr<IModuleCommandRTA> &rta)
{
    rta->addListener(this);
    m_RTA = rta;
}

void MaceCore::AddPathPlanningModule(const std::shared_ptr<IModuleCommandPathPlanning> &pathPlanning)
{
    pathPlanning->addListener(this);
    m_PathPlanning = pathPlanning;
}

void MaceCore::AddGroundStationModule(const std::shared_ptr<IModuleCommandGroundStation> &groundStation)
{
    bool serverStarted = groundStation->StartTCPServer();
    m_GroundStation = groundStation;
}


/////////////////////////////////////////////////////////////////////////
/// VEHICLE EVENTS
/////////////////////////////////////////////////////////////////////////
void MaceCore::NewConstructedVehicle(const void *sender, const std::shared_ptr<VehicleObject> &vehicleObject)
{
    counter_new_vehicle = counter_new_vehicle + 1;
    std::cout<<"The number of times a new vehicle has been constructed is: "<<counter_new_vehicle<<std::endl;

    IModuleCommandVehicle* vehicleModule = (IModuleCommandVehicle*)sender;
    int sendersID = 0;
    bool rtnValue = m_DataFusion->AddNewVehicle(vehicleObject,sendersID);

    m_VehicleObjectRequired.remove(sendersID);
    std::cout<<"The size of the list is now: "<<m_VehicleObjectRequired.size()<<std::endl;
    //vehicleModule->MarshalCommand(VehicleCommands::REMOVE_VEHICLE_OBJECT, sendersID);
    if(rtnValue == true){
        std::cout<<"A new vehicle has been added to the map. Notify everyone."<<std::endl;
    }
}

bool MaceCore::VehicleCheck(const int &vehicleID)
{
    std::list<int>::iterator it;
    for (it=m_VehicleObjectRequired.begin(); it != m_VehicleObjectRequired.end(); ++it)
    {
        if(*it == vehicleID)
        {
            return true;
        }
    }
    m_VehicleObjectRequired.push_back(vehicleID);
    return false;
}

void MaceCore::NewVehicleMessage(const void *sender, const TIME &time, const VehicleMessage &vehicleMessage)
{
    IModuleCommandVehicle* vehicleModule = (IModuleCommandVehicle*)sender;
    int sendersID = 0;
    bool rtnValue = m_DataFusion->HandleVehicleMessage(vehicleMessage,sendersID);
    if(rtnValue == false && this->VehicleCheck(sendersID) == false){
        std::cout<<"The value of counter is: "<<counter<<std::endl;
        counter = counter + 1;
        vehicleModule->MarshalCommand(VehicleCommands::CREATE_VEHICLE_OBJECT, sendersID);
    }else{
        std::string tmpString = "NA";
        //m_PathPlanning->MarshalCommand(PathPlanningCommands::UPDATED_POSITION_DYNAMICS, tmpString);
        //m_GroundStation->MarshalCommand(GroundStationCommands::UPDATED_POSITION_DYNAMICS, tmpString);
    }
}

void MaceCore::NewPositionDynamics(const void* sender, const TIME &time, const Eigen::Vector3d &pos, const Eigen::Vector3d &vel)
{
    IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
    std::string ID = m_VehiclePTRToID.at(vehicle);

    m_DataFusion->AddPositionDynamics(ID, time, pos, vel);

    m_RTA->MarshalCommand(RTACommands::UPDATED_POSITION_DYNAMICS, ID);
    m_PathPlanning->MarshalCommand(PathPlanningCommands::UPDATED_POSITION_DYNAMICS, ID);
    //m_GroundStation->MarshalCommand(GroundStationCommands::UPDATED_POSITION_DYNAMICS, ID);
}


void MaceCore::NewDynamicsDynamics(const void* sender, const TIME &time, const Eigen::Vector3d &attitude, const Eigen::Vector3d &attitudeRate)
{
    IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
    std::string ID = m_VehiclePTRToID.at(vehicle);

    m_DataFusion->AddAttitudeDynamics(ID, time, attitude, attitudeRate);

    m_RTA->MarshalCommand(RTACommands::UPDATED_ATTITUDE_DYNAMICS, ID);
    m_PathPlanning->MarshalCommand(PathPlanningCommands::UPDATED_ATTITUDE_DYNAMICS, ID);
    //m_GroundStation->MarshalCommand(GroundStationCommands::UPDATED_ATTITUDE_DYNAMICS, ID);
}


void MaceCore::NewVehicleLife(const void* sender, const TIME &time, const VehicleLife &life)
{
    IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
    std::string ID = m_VehiclePTRToID.at(vehicle);

    m_DataFusion->AddVehicleLife(ID, time, life);

    m_RTA->MarshalCommand(RTACommands::UPDATED_VEHICLE_LIFE, ID);
    m_PathPlanning->MarshalCommand(PathPlanningCommands::UPDATED_VEHICLE_LIFE, ID);
    //m_GroundStation->MarshalCommand(GroundStationCommands::UPDATED_VEHICLE_LIFE, ID);
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

    m_PathPlanning->NewVehicleTarget(vehicleID);
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

    m_RTA->UpdatedOccupancyMap();
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

    m_RTA->UpdatedOccupancyMap();
}


/////////////////////////////////////////////////////////////////////////
/// MACE COMMS EVENTS
/////////////////////////////////////////////////////////////////////////


} //END MaceCore Namespace
