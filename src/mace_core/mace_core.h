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

    //!
    //! \brief AddDataFusion Add MACE data pointer
    //! \param dataFusion Data fusion to add
    //!
    void AddDataFusion(const std::shared_ptr<MaceData> dataFusion);

    //!
    //! \brief AddVehicle Add vehicle to data fusion
    //! \param ID Vehicle ID to add
    //! \param vehicle Vehicle module pointer
    //!
    void AddVehicle(const std::string &ID, const std::shared_ptr<IModuleCommandVehicle> &vehicle);

    //!
    //! \brief RemoveVehicle Remove vehicle from data fusion
    //! \param ID Vehicle ID to remove
    //!
    void RemoveVehicle(const std::string &ID);


public: //The following functions add specific modules to connect to mace core

    //!
    //! \brief AddModule Add module to data fusion
    //! \param module Module to add
    //!
    void AddModule(const std::shared_ptr<ModuleBase> &module);

    //!
    //! \brief AddGroundStationModule Add ground station module
    //! \param groundStation Ground station module setup
    //!
    void AddGroundStationModule(const std::shared_ptr<IModuleCommandGroundStation> &groundStation);

    //!
    //! \brief AddExternalLink Add external link module
    //! \param externalLink External link module setup
    //!
    void AddExternalLink(const std::shared_ptr<IModuleCommandExternalLink> &externalLink);

    //!
    //! \brief AddPathPlanningModule Add path planning module
    //! \param pathPlanning Path planning module setup
    //!
    void AddPathPlanningModule(const std::shared_ptr<IModuleCommandPathPlanning> &pathPlanning);

    //!
    //! \brief AddROSModule Add ROS module
    //! \param ros ROS module setup
    //!
    void AddROSModule(const std::shared_ptr<IModuleCommandROS> &ros);

    //!
    //! \brief AddRTAModule Add RTA module
    //! \param rta RTA module setup
    //!
    void AddRTAModule(const std::shared_ptr<IModuleCommandRTA> &rta);

    //!
    //! \brief AddSensorsModule Add sensors module
    //! \param sensors Sensors module setup
    //!
    void AddSensorsModule(const std::shared_ptr<IModuleCommandSensors> &sensors);

public:

    //!
    //! \brief Add a module's topic chacteristic to the mace core
    //! \param sender Module that will be using topic
    //! \param topic Characteristic of topic
    //!
    void AddTopicCharacteristic(const ModuleBase *sender, const TopicCharacteristic &topic);

    //!
    //! \brief Subscribe Subscribe to a topic
    //! \param sender Module that will be using the topic
    //! \param topicName Topic name to subscribe to
    //! \param senderIDs Vector of IDs to subscribe
    //! \param components Vector of component names to subscribe
    //!
    virtual void Subscribe(ModuleBase* sender, const std::string &topicName, const std::vector<int> &senderIDs = {}, const std::vector<std::string> &components = {});

    //!
    //! \brief NewTopicDataValues Subscriber to new topic data values
    //! \param moduleFrom Module sending the new data values
    //! \param topicName Topic over which new values are present
    //! \param sender Sender module characteristic
    //! \param time Timestamp
    //! \param value Value of the new topic datagram
    //! \param target Target of the new topic datagram
    //!
    virtual void NewTopicDataValues(const ModuleBase* moduleFrom, const std::string &topicName, const ModuleCharacteristic &sender, const TIME &time, const TopicDatagram &value, const OptionalParameter<ModuleCharacteristic> &target = OptionalParameter<ModuleCharacteristic>());

    //!
    //! \brief NewTopicDataValues Subscriber to new topic data values
    //! \param moduleFrom Module sending the new data values
    //! \param topicName Topic over which the new values are present
    //! \param senderID Sender module ID
    //! \param time Timestamp
    //! \param value Value of the new topic datagram
    //!
    virtual void NewTopicDataValues(const ModuleBase* moduleFrom, const std::string &topicName, const int senderID, const TIME &time, const TopicDatagram &value);

public:
    /////////////////////////////////////////////////////////////////////////
    /// GENERAL MODULE EVENTS
    /////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Event_ForceVehicleDataSync Event to force a data dump of everything available to a vehicle
    //! \param sender Sender module
    //! \param targetSystemID Target ID of event
    //!
    virtual void Event_ForceVehicleDataSync(const ModuleBase *sender, const int &targetSystemID);

    //!
    //! \brief Event_IssueCommandSystemArm Event to trigger and ARM command
    //! \param sender Sender module
    //! \param command Arm/Disarm action
    //!
    virtual void Event_IssueCommandSystemArm(const ModuleBase* sender, const CommandItem::ActionArm &command);

    //!
    //! \brief Event_IssueCommandTakeoff Event to trigger a TAKEOFF command
    //! \param sender Sender module
    //! \param command Takeoff command
    //!
    virtual void Event_IssueCommandTakeoff(const ModuleBase* sender, const CommandItem::SpatialTakeoff &command);

    //!
    //! \brief Event_IssueCommandLand Event to trigger a LAND command
    //! \param sender Sender module
    //! \param command Land command
    //!
    virtual void Event_IssueCommandLand(const ModuleBase* sender, const CommandItem::SpatialLand &command);

    //!
    //! \brief Event_IssueCommandRTL Event to trigger an RTL command
    //! \param sender Sender module
    //! \param command RTL command
    //!
    virtual void Event_IssueCommandRTL(const ModuleBase* sender, const CommandItem::SpatialRTL &command);

    //!
    //! \brief Event_IssueMissionCommand Event to trigger a mission command
    //! \param sender Sender module
    //! \param command Mission command
    //!
    virtual void Event_IssueMissionCommand(const ModuleBase* sender, const CommandItem::ActionMissionCommand &command);

    //!
    //! \brief Event_ChangeSystemMode Event to trigger a mode change
    //! \param sender Sender module
    //! \param command Mode change command
    //!
    virtual void Event_ChangeSystemMode(const ModuleBase *sender, const CommandItem::ActionChangeMode &command);

    //!
    //! \brief Event_IssueGeneralCommand Event to trigger a general command
    //! \param sender Sender module
    //! \param command General command
    //!
    virtual void Event_IssueGeneralCommand(const ModuleBase* sender, const std::shared_ptr<CommandItem::AbstractCommandItem> &command);

    //!
    //! \brief Event_GetMission Event to trigger a "get mission" action
    //! \param sender Sender module
    //! \param key Mission key to fetch
    //!
    virtual void Event_GetMission(const void* sender, const MissionItem::MissionKey &key);

    //!
    //! \brief Event_GetOnboardMission Event to trigger a fetch of the current onboard mission for a vehicle
    //! \param sender Sender module
    //! \param systemID Vehicle ID whose mission we are asking for
    //! \param type Mission type
    //!
    virtual void Event_GetOnboardMission(const void* sender, const int &systemID, const MissionItem::MISSIONTYPE &type);

    //!
    //! \brief Event_GetCurrentMission Event to trigger a "get current mission" action
    //! \param sender Sender module
    //! \param systemID Vehicle ID whose mission we are asking for
    //!
    virtual void Event_GetCurrentMission(const void* sender, const int &systemID);

    //!
    //! \brief RequestClearVehicleMission Request a vehicle clear its onboard mission
    //! \param sender Sender module
    //! \param systemID Vehicle ID whose mission we want to clear
    //!
    virtual void RequestClearVehicleMission(const void* sender, const Data::SystemDescription &systemID);

    //!
    //! \brief RequestVehicleClearGuidedMission Request to clear a vehicle's guided mission
    //! \param sender Sender module
    //! \param vehicleID Vehicle ID whose guided mission we want to clear
    //!
    virtual void RequestVehicleClearGuidedMission(const void* sender, const int &vehicleID);

    //!
    //! \brief Event_GetHomePosition Event to trigger a fetch of a vehicle's home position
    //! \param sender Sender module
    //! \param vehicleID Vehicle ID whose home position we want
    //!
    virtual void Event_GetHomePosition(const void* sender, const int &vehicleID);

    //!
    //! \brief Event_SetHomePosition Event to trigger a set home position action
    //! \param sender Sender module
    //! \param vehicleHome New vehicle home position
    //!
    virtual void Event_SetHomePosition(const ModuleBase *sender, const CommandItem::SpatialHome &vehicleHome);

    //!
    //! \brief Event_SetGridSpacing Event to set a new grid spacing
    //! \param sender Sender module
    //! \param gridSpacing New grid spacing
    //!
    virtual void Event_SetGridSpacing(const void* sender, const double &gridSpacing);

    //!
    //! \brief Event_SetGlobalOrigin Event to set a new global origin
    //! \param sender Sender module
    //! \param globalHome New global origin position
    //!
    void Event_SetGlobalOrigin(const void* sender, const mace::pose::GeodeticPosition_3D &globalHome) override;


public:

    /////////////////////////////////////////////////////////////////////////
    /// SENSOR EVENTS
    /////////////////////////////////////////////////////////////////////////


    /////////////////////////////////////////////////////////////////////////////////////////
    /// SPECIFIC VEHICLE EVENTS: These events are associated from IModuleEventsVehicleVehicle
    /////////////////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief EventVehicle_NewConstructedVehicle New vehicle has been constructed
    //! \param sender Sender module
    //! \param newVehicleObserved New vehicle ID
    //!
    virtual void EventVehicle_NewConstructedVehicle(const void *sender, const int &newVehicleObserved);

    //!
    //! \brief EventVehicle_NewOnboardVehicleMission New onboard mission
    //! \param sender Sender module
    //! \param missionList New missionlist
    //!
    virtual void EventVehicle_NewOnboardVehicleMission(const ModuleBase *sender, const MissionItem::MissionList &missionList);

    //!
    //! \brief EventVehicle_MissionACK New mission ACK event
    //! \param sender Sender module
    //! \param ack Mission ack
    //!
    virtual void EventVehicle_MissionACK(const void *sender, const MissionItem::MissionACK &ack);

    //!
    //! \brief EventVehicle_REJECTProposedMission Event to trigger a rejected mission action
    //! \param sender Sender module
    //! \param key Rejected mission key
    //!
    virtual void EventVehicle_REJECTProposedMission(const void *sender, const MissionItem::MissionKey &key);

//    virtual void EventVehicle_ACKProposedMissionWChanges(const void *sender, const MissionItem::MissionKey &originalKey, const Data::MissionACK &ackCode, const MissionItem::MissionKey &newKey);

    ////////////////////////////////////////////////////////////////////////////////////////
    /// GENERAL VEHICLE EVENTS: These events are associated from IModuleEventsGeneralVehicle
    ////////////////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief GVEvents_NewHomePosition New home position
    //! \param sender Sender module
    //! \param vehicleHome New vehicle home
    //!
    virtual void GVEvents_NewHomePosition(const ModuleBase *sender, const CommandItem::SpatialHome &vehicleHome);

    //!
    //! \brief GVEvents_MissionExeStateUpdated New mission EXE state event
    //! \param sender Sender module
    //! \param missionKey Mission key corresponding to the new EXE state
    //! \param missionExeState New EXE state
    //!
    virtual void GVEvents_MissionExeStateUpdated(const void *sender, const MissionItem::MissionKey &missionKey, const Data::MissionExecutionState &missionExeState);

    //!
    //! \brief GVEvents_MissionItemAchieved Mission item achieved event
    //! \param sender Sender module
    //! \param achieved Mission item achieved data
    //!
    virtual void GVEvents_MissionItemAchieved(const void *sender, const MissionItem::MissionItemAchieved &achieved);

    //!
    //! \brief GVEvents_MissionItemCurrent New current mission item event
    //! \param sender Sender module
    //! \param current New current mission item
    //!
    virtual void GVEvents_MissionItemCurrent(const void *sender, const MissionItem::MissionItemCurrent &current);

    //!
    //! \brief GVEvents_NewSystemTime Emitted to alert the core that a module connected to a vehicle has an updated system time
    //! \param sender Sender module
    //! \param systemTime New system time
    //!
    virtual void GVEvents_NewSystemTime(const ModuleBase *sender, const DataGenericItem::DataGenericItem_SystemTime &systemTime);

    //!
    //! \brief ConfirmedOnboardVehicleMission Confirm onboard mission event
    //! \param sender Sender module
    //! \param missionKey Mission key to confirm
    //!
    virtual void ConfirmedOnboardVehicleMission(const void *sender, const MissionItem::MissionKey &missionKey);

    //!
    //! \brief NewCurrentVehicleMission New current mission event
    //! \param sender Sender module
    //! \param missionKey New mission key
    //!
    virtual void NewCurrentVehicleMission(const void *sender, const MissionItem::MissionKey &missionKey);

    /////////////////////////////////////////////////////////////////////////
    /// EXTERNAL LINK EVENTS
    /////////////////////////////////////////////////////////////////////////

    //!
    //! \brief ExternalEvent_UpdateRemoteID Update remote ID external event
    //! \param sender Sender module
    //! \param remoteID New remote ID
    //!
    virtual void ExternalEvent_UpdateRemoteID(const void *sender, const int &remoteID);

    //!
    //! \brief ExternalEvent_NewModule New module available to external link
    //! \param sender Sender module
    //! \param newModule New module
    //!
    virtual void ExternalEvent_NewModule(const void *sender, const ModuleCharacteristic &newModule);

    //!
    //! \brief ExternalEvent_FinishedRXMissionList Event signaling the receipt of a mission list
    //! \param sender Sender module
    //! \param missionList Received mission list
    //!
    virtual void ExternalEvent_FinishedRXMissionList(const void *sender, const MissionItem::MissionList &missionList);

    //!
    //! \brief ExternalEvent_MissionACK Event signaling a mission ACK
    //! \param sender Sender module
    //! \param missionACK Mission ACK
    //!
    virtual void ExternalEvent_MissionACK(const void* sender, const MissionItem::MissionACK &missionACK);

    //!
    //! \brief ExternalEvent_RequestingDataSync Request data sync of target's data
    //! \param sender Sender module
    //! \param targetID Target ID
    //!
    virtual void ExternalEvent_RequestingDataSync(const void *sender, const int &targetID);

    //!
    //! \brief ExternalEvent_NewOnboardMission New onboard mission event
    //! \param sender Sender module
    //! \param mission New mission key
    //!
    virtual void ExternalEvent_NewOnboardMission(const ModuleBase *sender, const MissionItem::MissionKey &mission);

    //!
    //! \brief MaceCore has been notified that a new boundary exists on a remote module.
    //!
    //!
    //! \param sender Module that is communicating with remote module
    //! \param data Data about boundary
    //!
    virtual void ExternalEvent_NewBoundary(const ModuleBase *sender, const NewBoundaryData &data);

    //!
    //! \brief ExternalEvent_FinishedRXBoundaryList Event signaling the receipt of a boundary list
    //! \param sender Sender module
    //! \param boundaryList New boundary list
    //!
    virtual void ExternalEvent_FinishedRXBoundaryList(const void *sender, const BoundaryItem::BoundaryList &boundaryList);

public:

    /////////////////////////////////////////////////////////////////////////
    /// BOUNDARY EVENTS
    /////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Function to fire when a new boundary of some kind was generated by a module
    //! \param sender Module that generated the boundary
    //! \param key Key indicating the characteristics of the boundary
    //! \param boundary Data for the boundary
    //!
    void Event_SetBoundary(const ModuleBase *sender, const BoundaryItem::BoundaryCharacterisic &key, const BoundaryItem::BoundaryList &boundary) override;

public:

    /////////////////////////////////////////////////////////////////////////
    /// GROUND STATION EVENTS
    /////////////////////////////////////////////////////////////////////////

    //!
    //! \brief RequestDummyFunction Test function
    //! \param sender Sender module
    //! \param vehicleID Vehicle ID
    //!
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

    //!
    //! \brief CommandNewVehicleMode Command a new vehicle mode change
    //! \param vehicleMode New vehicle mode string
    //!
    virtual void CommandNewVehicleMode(const std::string &vehicleMode);


public:

    /////////////////////////////////////////////////////////////////////////
    /// PATH PLANNING EVENTS
    /////////////////////////////////////////////////////////////////////////

    //!
    //! \brief EventPP_LoadOccupancyEnvironment Load a new occupancy map
    //! \param sender Sender module
    //! \param filePath Occupancy map file path
    //!
    void EventPP_LoadOccupancyEnvironment(const ModuleBase* sender, const std::string &filePath) override;

    //!
    //! \brief EventPP_LoadOctomapProperties Load octomap properties
    //! \param sender Sender module
    //! \param properties Octomap properties
    //!
    void EventPP_LoadOctomapProperties(const ModuleBase* sender, const mace::maps::OctomapSensorDefinition &properties) override;

    //!
    //! \brief EventPP_LoadMappingProjectionProperties Load map projection properties
    //! \param sender Sender module
    //! \param properties Map projection properties
    //!
    void EventPP_LoadMappingProjectionProperties(const ModuleBase* sender, const mace::maps::Octomap2DProjectionDefinition &properties) override;

    //!
    //! \brief EventPP_New2DOccupancyMap New compressed 2D occupancy map event
    //! \param sender Sender module
    //! \param map Occupancy map
    //!
    void EventPP_New2DOccupancyMap(const void* sender, const mace::maps::Data2DGrid<mace::maps::OccupiedResult> &map) override;

    //!
    //! \brief EventPP_NewDynamicMissionQueue New dynamic mission queue event
    //! \param sender Sender module
    //! \param queue New mission queue
    //!
    void EventPP_NewDynamicMissionQueue(const ModuleBase* sender, const TargetItem::DynamicMissionQueue &queue) override;

    //!
    //! \brief EventPP_NewPathFound New path found event
    //! \param sender Sender module
    //! \param path Path vector
    //!
    void EventPP_NewPathFound(const void* sender, const std::vector<mace::state_space::StatePtr> &path) override;

    //!
    //! \brief Event fired to indicate what planning horizon is being utilized by the path planning module
    //! \param horizon ID of the horizon being utilized
    //!
    virtual void PlanningHorizon(const std::string &horizon);

    //!
    //! \brief ReplaceVehicleCommands Replace vehicle commands with new commands
    //! \param vehicleID Target vehicle ID
    //! \param movementCommands New commands
    //!
    virtual void ReplaceVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands);

    //!
    //! \brief ReplaceAfterCurrentVehicleCommands Append vehicle commands after current vehicle commands
    //! \param vehicleID Target vehicle ID
    //! \param movementCommands New commands
    //!
    virtual void ReplaceAfterCurrentVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands);

    //!
    //! \brief AppendVehicleCommands Append vehicle commands to current vehicle commands
    //! \param vehicleID Target vehicle ID
    //! \param movementCommands New commands
    //!
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
    /// SENSOR MODULE EVENTS
    /////////////////////////////////////////////////////////////////////////

    //!
    //! \brief ROS_NewLaserScan New laser scan from ROS/Gazebo
    //! \param obj Point cloud object
    //! \param position Position of sensor
    //!
    void ROS_NewLaserScan(const octomap::Pointcloud &obj, const mace::pose::Position<mace::pose::CartesianPosition_3D>& position) override;

    //!
    //! \brief ROS_NewLaserScan New laser scan from ROS/Gazebo
    //! \param obj Point cloud object
    //! \param position Position of sensor
    //! \param orientation Orientation of sensor
    //!
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
                if(((CommandItem::std::shared_ptr<AbstractCommandItem>)Copy)->getTargetSystem() == 0)
                {
                    int ID = it->second->GetCharacteristic().ID;
                    ((CommandItem::std::shared_ptr<AbstractCommandItem>)Copy)->setTargetSystem(ID);
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
