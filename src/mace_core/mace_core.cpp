#include "mace_core.h"

#include <stdexcept>

namespace MaceCore
{

Mace_core::Mace_core()
{
}


/////////////////////////////////////////////////////////////////////////
/// CONFIGURE CORE
/////////////////////////////////////////////////////////////////////////


void Mace_core::AddDataFusion(const std::shared_ptr<MaceData> dataFusion)
{
    m_DataFusion = dataFusion;
}

void Mace_core::AddVehicle(const std::string &ID, const std::shared_ptr<IModuleCommandVehicle> &vehicle)
{
    if(m_VehicleIDToPtr.find(ID) != m_VehicleIDToPtr.cend())
        throw std::runtime_error("Vehicle ID already exists");

    m_VehicleIDToPtr.insert({ID, vehicle});

    m_DataFusion->AddVehicle(ID);

    m_RTA->NewVehicle(ID, vehicle->getModuleMetaData());
}


void Mace_core::RemoveVehicle(const std::string &ID)
{
    if(m_VehicleIDToPtr.find(ID) == m_VehicleIDToPtr.cend())
        throw std::runtime_error("Vehicle does not exists");

    m_VehicleIDToPtr.erase(m_VehicleIDToPtr.find(ID));

    m_DataFusion->RemoveVehicle(ID);

    m_RTA->RemoveVehicle(ID);
}


/////////////////////////////////////////////////////////////////////////
/// VEHICLE EVENTS
/////////////////////////////////////////////////////////////////////////

void Mace_core::NewPositionDynamics(const void* sender, const TIME &time, const VECTOR3D &pos, const VECTOR3D &vel)
{
    IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
    std::string rn = vehicle->ResourceName();

    m_DataFusion->AddPositionDynamics(rn, time, pos, vel);

    m_RTA->UpdatedPosition(rn);
    m_PathPlanning->UpdatedPosition(rn);
}


void Mace_core::NewDynamicsDynamics(const void* sender, const TIME &time, const VECTOR3D &attitude, const VECTOR3D &attitudeRate)
{
    IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
    std::string rn = vehicle->ResourceName();

    m_DataFusion->AddAttitudeDynamics(rn, time, attitude, attitudeRate);

    m_RTA->UpdateDynamicsState(rn);
    m_PathPlanning->UpdateDynamicsState(rn);
}


void Mace_core::NewVehicleLife(const void* sender, const TIME &time, const VehicleLife &life)
{
    IModuleCommandVehicle* vehicle = (IModuleCommandVehicle*)sender;
    std::string rn = vehicle->ResourceName();

    m_DataFusion->AddVehicleLife(rn, time, life);

    m_RTA->UpdatedVehicleLife(rn);
    m_PathPlanning->UpdatedVehicleLife(rn);
}


/////////////////////////////////////////////////////////////////////////
/// RTA EVENTS
/////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////
/// PATH PLANNING EVENTS
/////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////
/// MACE COMMS EVENTS
/////////////////////////////////////////////////////////////////////////


} //END MaceCore Namespace
