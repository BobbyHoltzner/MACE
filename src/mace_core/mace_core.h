#ifndef MACE_CORE_H
#define MACE_CORE_H

#include <string>
#include <map>
#include <memory>
#include <functional>

#include "mace_core_global.h"
#include "mace_data.h"

#include "i_module_command_external_link.h"
#include "i_module_command_ground_station.h"
#include "i_module_command_path_planning.h"
#include "i_module_command_RTA.h"
#include "i_module_command_sensors.h"
#include "i_module_command_vehicle.h"

#include "i_module_events_ground_station.h"
#include "i_module_events_path_planning.h"
#include "i_module_events_rta.h"
#include "i_module_events_sensors.h"
#include "i_module_events_vehicle.h"


#include "i_module_topic_events.h"

#include "topic.h"

namespace MaceCore
{

class MACE_CORESHARED_EXPORT MaceCore : public IModuleTopicEvents, public IModuleEventsVehicle, public IModuleEventsSensors, public IModuleEventsRTA, public IModuleEventsPathPlanning, public IModuleEventsGroundStation
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


public: //The following functions add specific modules to connect to mace core

    void AddGroundStationModule(const std::shared_ptr<IModuleCommandGroundStation> &groundStation);

    void AddExternalLink(const std::shared_ptr<IModuleCommandExternalLink> &externalLink);

    void AddPathPlanningModule(const std::shared_ptr<IModuleCommandPathPlanning> &pathPlanning);

    void AddRTAModule(const std::shared_ptr<IModuleCommandRTA> &rta);

    void AddSensorsModule(const std::shared_ptr<IModuleCommandSensors> &sensors);

public:

    void AddTopic(const std::string &topicName, const TopicStructure &topic);


    virtual void Subscribe(ModuleBase* sender, const std::string &topicName, const std::vector<int> &senderIDs = {}, const std::vector<std::string> &components = {});

    virtual void NewTopicDataValues(const std::string &topicName, const int senderID, const TIME &time, const TopicDatagram &value);

public:
    /////////////////////////////////////////////////////////////////////////
    /// GENERAL MODULE EVENTS
    /////////////////////////////////////////////////////////////////////////

    virtual void RequestVehicleArm(const void* sender, const MissionItem::ActionArm &arm);
    virtual void RequestVehicleMode(const void* sender, const MissionItem::ActionChangeMode &changeMode);
    virtual void RequestCurrentVehicleMission(const void* sender, const int &vehicleID);
    virtual void RequestVehicleClearAutoMission(const void* sender, const int &vehicleID);
    virtual void RequestVehicleClearGuidedMission(const void* sender, const int &vehicleID);

    virtual void RequestVehicleHomePosition(const void* sender, const int &vehicleID);
    virtual void SetVehicleHomePosition(const void* sender, const MissionItem::SpatialHome &vehicleHome);

    virtual void UpdateGlobalOriginPosition(const void* sender, const MissionItem::SpatialHome &globalHome);

public:

    /////////////////////////////////////////////////////////////////////////
    /// SENSOR EVENTS
    /////////////////////////////////////////////////////////////////////////


    /////////////////////////////////////////////////////////////////////////
    /// VEHICLE EVENTS
    /////////////////////////////////////////////////////////////////////////

    virtual void NewConstructedVehicle(const void* sender, const int &newVehicleObserved);
    virtual void NewVehicleHomePosition(const void *sender, const MissionItem::SpatialHome &vehicleHome);


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
    /// GROUND STATION EVENTS
    /////////////////////////////////////////////////////////////////////////


    //!
    //! \brief Event fired when a new list of targets are produced for a specific vehicle
    //! \param vehicleID Vechile new targets are to be applied to
    //! \param target List of positional targets
    //!
    virtual void GroundStationEvent();

    virtual void CommandNewVehicleMode(const std::string &vehicleMode);


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
    std::unordered_map<std::string, TopicStructure> m_Topics;
    std::unordered_map<std::string, std::vector<ModuleBase*>> m_TopicNotifier;

    std::map<int, IModuleCommandVehicle*> m_VehicleIDToPort;

    std::map<std::string, IModuleCommandVehicle*> m_VehicleIDToPtr;
    std::map<IModuleCommandVehicle*, std::string> m_VehiclePTRToID;


    std::shared_ptr<IModuleCommandGroundStation> m_GroundStation;
    std::shared_ptr<IModuleCommandPathPlanning> m_PathPlanning;
    std::shared_ptr<IModuleCommandSensors> m_Sensors;
    std::shared_ptr<IModuleCommandRTA> m_RTA;

    std::shared_ptr<MaceData> m_DataFusion;
};

} //END MaceCore Namespace

#endif // MACE_CORE_H
