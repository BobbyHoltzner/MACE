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

namespace MaceCore
{

class MACE_CORESHARED_EXPORT MaceCore : public IModuleEventsVehicle, public IModuleEventsRTA, public IModuleEventsPathPlanning
{

public:
    MaceCore();


public:

    /////////////////////////////////////////////////////////////////////////
    /// CONFIGURE CORE
    /////////////////////////////////////////////////////////////////////////

    void AddDataFusion(const std::shared_ptr<MaceData> dataFusion);

    void AddVehicle(const std::string &ID, const std::shared_ptr<IModuleCommandVehicle> &vehicle);

    void RemoveVehicle(const std::string &ID);

    void AddRTAModule(const std::shared_ptr<IModuleCommandRTA> &rta);

    void AddPathPlanningModule(const std::shared_ptr<IModuleCommandPathPlanning> &pathPlanning);

public:

    /////////////////////////////////////////////////////////////////////////
    /// VEHICLE EVENTS
    /////////////////////////////////////////////////////////////////////////

    virtual void NewPositionDynamics(const void* sender, const TIME &time, const Eigen::Vector3d &position, const Eigen::Vector3d &attitude);

    virtual void NewDynamicsDynamics(const void* sender, const TIME &time, const Eigen::Vector3d &attitude, const Eigen::Vector3d &attitudeRate);

    virtual void NewVehicleLife(const void* sender, const TIME &time, const VehicleLife &life);

public:

    /////////////////////////////////////////////////////////////////////////
    /// RTA EVENTS
    /////////////////////////////////////////////////////////////////////////


    //!
    //! \brief Event fired when a new list of targets are produced for a specific vehicle
    //! \param vehicleID Vechile new targets are to be applied to
    //! \param target List of positional targets
    //!
    virtual void NewVehicleTargets(const std::string &vehicleID, const std::vector<Eigen::Vector3d> &target);

public:

    /////////////////////////////////////////////////////////////////////////
    /// PATH PLANNING EVENTS
    /////////////////////////////////////////////////////////////////////////


    virtual void ReplaceVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands);

    virtual void ReplaceAfterCurrentVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands);

    virtual void AppendVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands);

public:

    /////////////////////////////////////////////////////////////////////////
    /// MACE COMMS EVENTS
    /////////////////////////////////////////////////////////////////////////

private:


    std::map<std::string, std::shared_ptr<IModuleCommandVehicle>> m_VehicleIDToPtr;

    std::map<IModuleCommandVehicle*, std::string> m_VehiclePTRToID;


    std::shared_ptr<IModuleCommandRTA> m_RTA;

    std::shared_ptr<IModuleCommandPathPlanning> m_PathPlanning;


    std::shared_ptr<MaceData> m_DataFusion;
};

} //END MaceCore Namespace

#endif // MACE_CORE_H
