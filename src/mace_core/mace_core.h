#ifndef MACE_CORE_H
#define MACE_CORE_H
#include <QtGlobal>

#include <iostream>

#include <string>
#include <map>
#include <memory>
#include <functional>
#include <mutex>

#include "mace_core_global.h"
#include "mace_data.h"
#include "base/geometry/cell_2DC.h"

#include "i_module_command_external_link.h"
#include "i_module_command_ground_station.h"
#include "i_module_command_path_planning.h"
#include "i_module_command_ROS.h"
#include "i_module_command_RTA.h"
#include "i_module_command_sensors.h"
#include "i_module_command_vehicle.h"

#include "i_module_events_external_link.h"
#include "i_module_events_ground_station.h"
#include "i_module_events_path_planning.h"
#include "i_module_events_ROS.h"
#include "i_module_events_rta.h"
#include "i_module_events_sensors.h"
#include "i_module_events_vehicle.h"

#include "i_module_topic_events.h"

#include "topic.h"

#include "octomap/octomap.h"
#include "octomap/OcTree.h"

#include "data_generic_state_item/positional_aid.h"
#include "data_generic_command_item/command_item_components.h"
//#include "data_generic_command_item/boundary_items/boundary_key.h"
//#include "data_generic_command_item/boundary_items/boundary_type.h"
//#include "data_generic_command_item/boundary_items/boundary_list.h"

namespace MaceCore
{

class MACE_CORESHARED_EXPORT MaceCore : public IModuleTopicEvents, public IModuleEventsVehicle, public IModuleEventsSensors, public IModuleEventsRTA, public IModuleEventsPathPlanning, public IModuleEventsROS, public IModuleEventsGroundStation, public IModuleEventsExternalLink
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

    void AddModule(const std::shared_ptr<ModuleBase> &module);

    void AddGroundStationModule(const std::shared_ptr<IModuleCommandGroundStation> &groundStation);

    void AddExternalLink(const std::shared_ptr<IModuleCommandExternalLink> &externalLink);

    void AddPathPlanningModule(const std::shared_ptr<IModuleCommandPathPlanning> &pathPlanning);

    void AddROSModule(const std::shared_ptr<IModuleCommandROS> &ros);

    void AddRTAModule(const std::shared_ptr<IModuleCommandRTA> &rta);

    void AddSensorsModule(const std::shared_ptr<IModuleCommandSensors> &sensors);

public:

    //!
    //! \brief Add a module's topic chacteristic to the mace core
    //! \param sender Module that will be using topic
    //! \param topic Characteristic of topic
    //!
    void AddTopicCharacteristic(const ModuleBase *sender, const TopicCharacteristic &topic);


    virtual void Subscribe(ModuleBase* sender, const std::string &topicName, const std::vector<int> &senderIDs = {}, const std::vector<std::string> &components = {});

    virtual void NewTopicDataValues(const ModuleBase* moduleFrom, const std::string &topicName, const ModuleCharacteristic &sender, const TIME &time, const TopicDatagram &value, const OptionalParameter<ModuleCharacteristic> &target = OptionalParameter<ModuleCharacteristic>());

    virtual void NewTopicDataValues(const ModuleBase* moduleFrom, const std::string &topicName, const int senderID, const TIME &time, const TopicDatagram &value);

public:
    /////////////////////////////////////////////////////////////////////////
    /// GENERAL MODULE EVENTS
    /////////////////////////////////////////////////////////////////////////

    virtual void Event_ForceVehicleDataSync(const ModuleBase *sender, const int &targetSystemID);
    virtual void Event_IssueCommandSystemArm(const ModuleBase* sender, const CommandItem::ActionArm &command);
    virtual void Event_IssueCommandTakeoff(const ModuleBase* sender, const CommandItem::SpatialTakeoff &command);
    virtual void Event_IssueCommandLand(const ModuleBase* sender, const CommandItem::SpatialLand &command);
    virtual void Event_IssueCommandRTL(const ModuleBase* sender, const CommandItem::SpatialRTL &command);
    virtual void Event_IssueMissionCommand(const ModuleBase* sender, const CommandItem::ActionMissionCommand &command);
    virtual void Event_ChangeSystemMode(const ModuleBase *sender, const CommandItem::ActionChangeMode &command);
    virtual void Event_IssueGeneralCommand(const ModuleBase* sender, const std::shared_ptr<CommandItem::AbstractCommandItem> &command);

    virtual void Event_GetMission(const void* sender, const MissionItem::MissionKey &key);
    virtual void Event_GetOnboardMission(const void* sender, const int &systemID, const MissionItem::MISSIONTYPE &type);
    virtual void Event_GetCurrentMission(const void* sender, const int &systemID);

    virtual void RequestClearVehicleMission(const void* sender, const Data::SystemDescription &systemID);
    virtual void RequestVehicleClearGuidedMission(const void* sender, const int &vehicleID);

    virtual void Event_GetHomePosition(const void* sender, const int &vehicleID);
    virtual void Event_SetHomePosition(const ModuleBase *sender, const CommandItem::SpatialHome &vehicleHome);

    virtual void Event_SetGridSpacing(const void* sender, const double &gridSpacing);


    void Event_SetGlobalOrigin(const void* sender, const mace::pose::GeodeticPosition_3D &globalHome) override;

    void Event_SetBoundary(const ModuleBase* sender, const BoundaryItem::BoundaryList &boundary) override;

public:

    /////////////////////////////////////////////////////////////////////////
    /// SENSOR EVENTS
    /////////////////////////////////////////////////////////////////////////


    /////////////////////////////////////////////////////////////////////////////////////////
    /// SPECIFIC VEHICLE EVENTS: These events are associated from IModuleEventsVehicleVehicle
    /////////////////////////////////////////////////////////////////////////////////////////

    virtual void EventVehicle_NewConstructedVehicle(const void *sender, const int &newVehicleObserved);

    virtual void EventVehicle_NewOnboardVehicleMission(const ModuleBase *sender, const MissionItem::MissionList &missionList);

    virtual void EventVehicle_MissionACK(const void *sender, const MissionItem::MissionACK &ack);

    virtual void EventVehicle_REJECTProposedMission(const void *sender, const MissionItem::MissionKey &key);

//    virtual void EventVehicle_ACKProposedMissionWChanges(const void *sender, const MissionItem::MissionKey &originalKey, const Data::MissionACK &ackCode, const MissionItem::MissionKey &newKey);

    ////////////////////////////////////////////////////////////////////////////////////////
    /// GENERAL VEHICLE EVENTS: These events are associated from IModuleEventsGeneralVehicle
    ////////////////////////////////////////////////////////////////////////////////////////
    virtual void GVEvents_NewHomePosition(const ModuleBase *sender, const CommandItem::SpatialHome &vehicleHome);
    virtual void GVEvents_MissionExeStateUpdated(const void *sender, const MissionItem::MissionKey &missionKey, const Data::MissionExecutionState &missionExeState);
    virtual void GVEvents_MissionItemAchieved(const void *sender, const MissionItem::MissionItemAchieved &achieved);
    virtual void GVEvents_MissionItemCurrent(const void *sender, const MissionItem::MissionItemCurrent &current);
    virtual void ConfirmedOnboardVehicleMission(const void *sender, const MissionItem::MissionKey &missionKey);
    virtual void NewCurrentVehicleMission(const void *sender, const MissionItem::MissionKey &missionKey);

    /////////////////////////////////////////////////////////////////////////
    /// EXTERNAL LINK EVENTS
    /////////////////////////////////////////////////////////////////////////
    virtual void ExternalEvent_UpdateRemoteID(const void *sender, const int &remoteID);
    virtual void ExternalEvent_NewModule(const void *sender, const ModuleCharacteristic &newModule);


    virtual void ExternalEvent_FinishedRXMissionList(const void *sender, const MissionItem::MissionList &missionList);

    virtual void ExternalEvent_MissionACK(const void* sender, const MissionItem::MissionACK &missionACK);
    virtual void ExternalEvent_RequestingDataSync(const void *sender, const int &targetID);

    virtual void ExternalEvent_NewOnboardMission(const ModuleBase *sender, const MissionItem::MissionKey &mission);

    virtual void ExternalEvent_FinishedRXBoundaryList(const void *sender, const BoundaryItem::BoundaryList &boundaryList);

public:

    /////////////////////////////////////////////////////////////////////////
    /// RTA EVENTS
    /////////////////////////////////////////////////////////////////////////

    void Event_SetResourceBoundary(const ModuleBase *sender, const BoundaryItem::BoundaryList &boundary) override;

public:

    /////////////////////////////////////////////////////////////////////////
    /// GROUND STATION EVENTS
    /////////////////////////////////////////////////////////////////////////

    virtual void RequestDummyFunction(const void* sender, const int &vehicleID);

    //!
    //! \brief Event_UploadMission method calls the appropriate handling operations to migrate the proposed
    //! mission list to the appropriate vehicle module for handling.
    //! \param sender
    //! \param missionList
    //!
    virtual void GSEvent_UploadMission(const void* sender, const MissionItem::MissionList &missionList);

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

    void EventPP_LoadOccupancyEnvironment(const ModuleBase* sender, const std::string &filePath) override;

    void Event_SetOperationalBoundary(const ModuleBase *sender, const BoundaryItem::BoundaryList &boundary) override;

    //!
    //! \brief Event fired to indicate what planning horizon is being utilized by the path planning module
    //! \param horizon ID of the horizon being utilized
    //!
    virtual void PlanningHorizon(const std::string &horizon);

    virtual void ReplaceVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands);

    virtual void ReplaceAfterCurrentVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands);

    virtual void AppendVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands);

    void EventPP_New2DOccupancyMap(const void* sender, const mace::maps::Data2DGrid<mace::maps::OccupiedResult> &map) override;

    void EventPP_NewPathFound(const void* sender, const std::vector<mace::state_space::StatePtr> &path) override;

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
    /// SENSOR MODULE EVENTS
    /////////////////////////////////////////////////////////////////////////
    void ROS_NewLaserScan(const octomap::Pointcloud &obj, const mace::pose::Position<mace::pose::CartesianPosition_3D>& position) override;

    void ROS_NewLaserScan(const octomap::Pointcloud &obj, const mace::pose::Position<mace::pose::CartesianPosition_3D>& position, const mace::pose::Orientation_3D& orientation) override;
public:

    /////////////////////////////////////////////////////////////////////////
    /// MACE COMMS EVENTS
    /////////////////////////////////////////////////////////////////////////

private:


    /**
     * @brief This function marshals a command to be sent to a vehicle
     *
     * The vehicle can either be attached locally or through external module
     * @param vehicleID ID of vehicle, if zero then message is to be broadcast to all vehicles.
     * @param vehicleCommand Command if vehicle attached locally
     * @param externalCommand Command if vehicle attached remotly
     * @param data Data to send to given vehicle
     */
    template <typename T>
    void MarshalCommandToVehicle(int vehicleID, VehicleCommands vehicleCommand, ExternalLinkCommands externalCommand, const T& data, OptionalParameter<ModuleCharacteristic> sender = OptionalParameter<ModuleCharacteristic>())
    {
        //transmit to all
        if(vehicleID == 0) {

            for(auto it = m_VehicleIDToPort.begin() ; it != m_VehicleIDToPort.end() ; ++it)
            {
                //don't resend to sender.
                if(sender.IsSet() && it->second->GetCharacteristic() == sender())
                {
                    continue;
                }
                /*
                T *Copy = new T(data);
                if(((CommandItem::AbstractCommandItem *)Copy)->getTargetSystem() == 0)
                {
                    int ID = it->second->GetCharacteristic().ID;
                    ((CommandItem::AbstractCommandItem *)Copy)->setTargetSystem(ID);
                }
                */
                it->second->MarshalCommand(vehicleCommand, data);
            }

            for(auto it = m_ExternalLink.begin() ; it != m_ExternalLink.end() ; ++it)
            {
                //don't resend to sender.
                if(sender.IsSet() && (*it)->GetCharacteristic() == sender())
                {
                    continue;
                }
                (*it)->MarshalCommand(externalCommand, data, sender);
            }
        }
        else {
            if(m_VehicleIDToPort.find(vehicleID) != m_VehicleIDToPort.cend()){
                m_VehicleIDToPort.at(vehicleID)->MarshalCommand(vehicleCommand, data);
            }

            else if(m_ExternalLinkIDToPort.find(vehicleID) != m_ExternalLinkIDToPort.cend()){
                m_ExternalLinkIDToPort.at(vehicleID)->MarshalCommand(externalCommand, data, sender);
            }
            else {
                throw std::runtime_error("Unknown vehicle");
            }
        }
    }

private:
    mutable std::mutex m_VehicleMutex;

    std::unordered_map<const ModuleBase*, std::unordered_map<std::string, TopicCharacteristic>> m_TopicsToReceive;
    std::unordered_map<std::string, std::vector<ModuleBase*>> m_TopicNotifier;

    std::map<int, IModuleCommandVehicle*> m_VehicleIDToPort;

    std::map<std::string, IModuleCommandVehicle*> m_VehicleIDToPtr;
    std::map<IModuleCommandVehicle*, std::string> m_VehiclePTRToID;

    std::list<std::shared_ptr<IModuleCommandExternalLink>> m_ExternalLink;
    std::map<int, IModuleCommandExternalLink*> m_ExternalLinkIDToPort;

    std::shared_ptr<IModuleCommandGroundStation> m_GroundStation;
    std::shared_ptr<IModuleCommandPathPlanning> m_PathPlanning;
    std::shared_ptr<IModuleCommandROS> m_ROS;
    std::shared_ptr<IModuleCommandSensors> m_Sensors;
    std::shared_ptr<IModuleCommandRTA> m_RTA;

    std::shared_ptr<MaceData> m_DataFusion;
};

} //END MaceCore Namespace

#endif // MACE_CORE_H
