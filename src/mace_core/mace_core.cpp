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

    m_RTA->NewVehicle(ID, vehicle->getModuleMetaData());
}


void MaceCore::RemoveVehicle(const std::string &ID)
{
    if(m_VehicleIDToPtr.find(ID) == m_VehicleIDToPtr.cend())
        throw std::runtime_error("Vehicle does not exists");

    m_VehicleIDToPtr.erase(m_VehicleIDToPtr.find(ID));

    m_DataFusion->RemoveVehicle(ID);

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

    m_RTA->UpdatedPosition(rn);
    m_PathPlanning->UpdatedPosition(rn);
}


void MaceCore::NewDynamicsDynamics(const void* sender, const TIME &time, const Eigen::Vector3d &attitude, const Eigen::Vector3d &attitudeRate)
{
    IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
    std::string rn = m_VehiclePTRToID.at(vehicle);

    m_DataFusion->AddAttitudeDynamics(rn, time, attitude, attitudeRate);

    m_RTA->UpdateDynamicsState(rn);
    m_PathPlanning->UpdateDynamicsState(rn);
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

/////////////////////////////////////////////////////////////////////////
/// MACE COMMS EVENTS
/////////////////////////////////////////////////////////////////////////


} //END MaceCore Namespace
