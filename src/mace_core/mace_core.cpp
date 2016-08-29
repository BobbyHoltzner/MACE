#include "mace_core.h"

#include <stdexcept>

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

    m_VehicleIDToPtr.insert({ID, vehicle});

    m_DataFusion->AddVehicle(ID);

    if(m_RTA != NULL)
        m_RTA->NewVehicle(ID, vehicle->getModuleMetaData());
}


void MaceCore::RemoveVehicle(const std::string &ID)
{
    if(m_VehicleIDToPtr.find(ID) == m_VehicleIDToPtr.cend())
        throw std::runtime_error("Vehicle does not exists");

    m_VehicleIDToPtr.erase(m_VehicleIDToPtr.find(ID));

    m_DataFusion->RemoveVehicle(ID);

    if(m_RTA != NULL)
        m_RTA->RemoveVehicle(ID);
}


void MaceCore::AddRTAModule(const std::shared_ptr<IModuleCommandRTA> &rta)
{
    m_RTA = rta;
}

void MaceCore::AddPathPlanningModule(const std::shared_ptr<IModuleCommandPathPlanning> &pathPlanning)
{
    m_PathPlanning = pathPlanning;
}


/////////////////////////////////////////////////////////////////////////
/// VEHICLE EVENTS
/////////////////////////////////////////////////////////////////////////

void MaceCore::NewPositionDynamics(const void* sender, const TIME &time, const Eigen::Vector3d &pos, const Eigen::Vector3d &vel)
{
    IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
    std::string rn = m_VehiclePTRToID.at(vehicle);

    m_DataFusion->AddPositionDynamics(rn, time, pos, vel);

    m_RTA->UpdatedPositionDynamics(rn);
    m_PathPlanning->UpdatedPositionDynamics(rn);
}


void MaceCore::NewDynamicsDynamics(const void* sender, const TIME &time, const Eigen::Vector3d &attitude, const Eigen::Vector3d &attitudeRate)
{
    IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
    std::string rn = m_VehiclePTRToID.at(vehicle);

    m_DataFusion->AddAttitudeDynamics(rn, time, attitude, attitudeRate);

    m_RTA->UpdateAttitudeDynamics(rn);
    m_PathPlanning->UpdateAttitudeDynamics(rn);
}


void MaceCore::NewVehicleLife(const void* sender, const TIME &time, const VehicleLife &life)
{
    IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
    std::string rn = m_VehiclePTRToID.at(vehicle);

    m_DataFusion->AddVehicleLife(rn, time, life);

    m_RTA->UpdatedVehicleLife(rn);
    m_PathPlanning->UpdatedVehicleLife(rn);
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

    m_VehicleIDToPtr.at(vehicleID)->FollowNewCommands();
}

void MaceCore::ReplaceAfterCurrentVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands)
{
    m_DataFusion->setVehicleDynamicsCommands(vehicleID, movementCommands);

    m_VehicleIDToPtr.at(vehicleID)->FinishAndFollowNewCommands();
}

void MaceCore::AppendVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands)
{
    std::vector<FullVehicleDynamics> commands = m_DataFusion->getVehicleDynamicsCommands(vehicleID);
    commands.insert(commands.end(), movementCommands.begin(), movementCommands.end());
    m_DataFusion->setVehicleDynamicsCommands(vehicleID, commands);

    m_VehicleIDToPtr.at(vehicleID)->CommandsAppended();
}

//!
//! \brief Event fired when a new occupancy map to be invoked when PathPlanning module generates a new occupancy map.
//! \param occupancyMap New occupancy map
//!
void MaceCore::NewOccupancyMap(const Eigen::MatrixXd &occupancyMap)
{
    m_DataFusion->ReplaceOccupanyMap(occupancyMap);

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
        OperateOnMatrixd(mat, commands);
    };

    m_DataFusion->OperateOnOccupanyMap(func);

    m_RTA->UpdatedOccupancyMap();
}


/////////////////////////////////////////////////////////////////////////
/// MACE COMMS EVENTS
/////////////////////////////////////////////////////////////////////////


} //END MaceCore Namespace
