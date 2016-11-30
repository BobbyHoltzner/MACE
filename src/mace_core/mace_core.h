#ifndef MACE_CORE_H
#define MACE_CORE_H

#include <string>
#include <map>
#include <memory>
#include <functional>

#include "mace_core_global.h"
#include "mace_data.h"
#include "vehicle_object.h"

#include "i_module_command_vehicle.h"
#include "i_module_command_RTA.h"
#include "i_module_command_path_planning.h"

#include "i_module_events_vehicle.h"
#include "i_module_events_rta.h"
#include "i_module_events_path_planning.h"

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

    virtual void NewConstructedVehicle(const void* sender, std::shared_ptr<VehicleObject> vehicleObject);

    virtual void NewVehicleMessage(const void* sender, const TIME &time, const VehicleMessage &vehicleMessage);

    virtual void TestNewVehicleMessage(const void* sender, const TIME &time, std::function<std::vector<std::string>(VehicleObject*)> vehicleFunction);

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


    //!
    //! \brief Event fired to indicate what planning horizon is being utilized by the path planning module
    //! \param horizon ID of the horizon being utilized
    //!
    virtual void PlanningHorizon(const std::string &horizon);

    virtual void ReplaceVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands);

    virtual void ReplaceAfterCurrentVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands);

    virtual void AppendVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands);


    //!
    //! \brief Event fired when a new occupancy map to be invoked when PathPlanning module generates a new occupancy map.
    //! \param occupancyMap New occupancy map
    //!
    virtual void NewOccupancyMap(const Eigen::MatrixXd &occupancyMap);


    //!
    //! \brief Event fired when the PathPlanning modules determines that a set of cells should be modified on the occupancy map.
    //!
    //! This event may be faster than NewOccupancyMap when the matrix is large and the modifcations are sparse
    //! \param commands List of cells to modify
    //!
    virtual void ReplaceOccupancyMapCells(const std::vector<MatrixCellData<double>> &commands);


public:

    /////////////////////////////////////////////////////////////////////////
    /// MACE COMMS EVENTS
    /////////////////////////////////////////////////////////////////////////

private:
    int counter;
    bool insertFlag;
    std::map<int, std::shared_ptr<VehicleObject>> m_VehicleData;
    std::map<int, IModuleCommandVehicle*> m_VehicleIDToPort;
    std::map<IModuleCommandVehicle*, int> m_PortToVehicleID;

    std::map<std::string, IModuleCommandVehicle*> m_VehicleIDToPtr;

    std::map<IModuleCommandVehicle*, std::string> m_VehiclePTRToID;


    std::shared_ptr<IModuleCommandRTA> m_RTA;

    std::shared_ptr<IModuleCommandPathPlanning> m_PathPlanning;


    std::shared_ptr<MaceData> m_DataFusion;
};

} //END MaceCore Namespace

#endif // MACE_CORE_H
