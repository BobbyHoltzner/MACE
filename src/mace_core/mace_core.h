#ifndef MACE_CORE_H
#define MACE_CORE_H

#include "mace_core_global.h"

#include "mace_data.h"

#include "i_module_command_vehicle.h"
#include "i_module_command_RTA.h"
#include "i_module_command_path_planning.h"

#include "i_module_events_vehicle.h"
#include "i_module_events_rta.h"
#include "i_module_events_path_planning.h"

#include <string>
#include <map>
#include <memory>

class MACE_CORESHARED_EXPORT Mace_core : public IModuleEventsVehicle, public IModuleEventsRTA, public IModuleEventsPathPlanning
{

public:
    Mace_core();


public:

    /////////////////////////////////////////////////////////////////////////
    /// CONFIGURE CORE
    /////////////////////////////////////////////////////////////////////////

    void AddDataFusion(const std::shared_ptr<MaceData> dataFusion);

    void AddVehicle(const std::string &ID, const std::shared_ptr<IModuleCommandVehicle> &vehicle);

    void RemoveVehicle(const std::string &ID);

public:

    /////////////////////////////////////////////////////////////////////////
    /// VEHICLE EVENTS
    /////////////////////////////////////////////////////////////////////////

    virtual void NewPositionDynamics(const void* sender, const TIME &time, const VECTOR3D &position, const VECTOR3D &attitude);

    virtual void NewDynamicsDynamics(const void* sender, const TIME &time, const VECTOR3D &attitude, const VECTOR3D &attitudeRate);

    virtual void NewVehicleLife(const void* sender, const TIME &time, const VehicleLife &life);

public:

    /////////////////////////////////////////////////////////////////////////
    /// RTA EVENTS
    /////////////////////////////////////////////////////////////////////////


public:

    /////////////////////////////////////////////////////////////////////////
    /// PATH PLANNING EVENTS
    /////////////////////////////////////////////////////////////////////////


public:

    /////////////////////////////////////////////////////////////////////////
    /// MACE COMMS EVENTS
    /////////////////////////////////////////////////////////////////////////

private:


    std::map<std::string, std::shared_ptr<IModuleCommandVehicle>> m_VehicleIDToPtr;


    std::shared_ptr<IModuleCommandRTA> m_RTA;

    std::shared_ptr<IModuleCommandPathPlanning> m_PathPlanning;


    std::shared_ptr<MaceData> m_DataFusion;
};

#endif // MACE_CORE_H
