#ifndef MODULE_EXTERNAL_LINK_H
#define MODULE_EXTERNAL_LINK_H

#include "module_external_link_global.h"

#include <sstream>
#include <iostream>
#include <stdint.h>
#include <chrono>
#include <functional>

#include "spdlog/spdlog.h"
#include "mace.h"

#include "common/common.h"

#include "commsMACEHelper/comms_mace_helper.h"

#include "mace_core/i_module_topic_events.h"
#include "mace_core/i_module_command_external_link.h"
#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data_vehicle_sensors/components.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_command_item/command_item_components.h"
#include "data_generic_command_item_topic/command_item_topic_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

#include "controllers/heartbeat_controller_externallink.h"

#include "controllers/generic_controller.h"


#include "controllers/commands/command_land.h"
#include "controllers/commands/command_takeoff.h"
#include "controllers/commands/command_arm.h"
#include "controllers/commands/command_rtl.h"
#include "controllers/commands/command_mission_item.h"
#include "controllers/controller_system_mode.h"
#include "controllers/controller_home.h"
#include "controllers/controller_mission.h"


#include "mace_core/module_characteristics.h"

#include "data/topic_components/altitude.h"
#include "data/topic_components/position_global.h"
#include "data/topic_components/position_local.h"
#include "data/topic_components/topic_component_void.h"
#include "data/topic_components/topic_component_string.h"

#include "base_topic/vehicle_topics.h"


class MODULE_EXTERNAL_LINKSHARED_EXPORT ModuleExternalLink :
        public MaceCore::IModuleCommandExternalLink,
        public CommsMACEHelper,
        public ExternalLink::HeartbeatController_Interface,
        public Controllers::IMessageNotifier<mace_message_t>
{


private:

    /**
     * @brief BroadcastLogicToAllVehicles
     * @param vehicleID VehicleID to send function to. If set to 0 then send to all local vehicles
     * @param func Function to call
     */
    void BroadcastLogicToAllVehicles(int vehicleID, const std::function<void(int)> &func)
    {
        if(vehicleID == 0)
        {
            std::vector<int> vehicleIDs;
            this->getDataObject()->GetLocalVehicles(vehicleIDs);
            for(auto it = vehicleIDs.begin() ; it != vehicleIDs.end() ; ++it)
            {
                func(*it);
            }
        }
        else
        {
            func(vehicleID);
        }
    }


    enum class ContainedControllers
    {
        MISSION_DOWNLOAD,
        MISSION_UPLOAD
    };

public:

    ModuleExternalLink();

    ~ModuleExternalLink();

    virtual std::vector<MaceCore::TopicCharacteristic> GetEmittedTopics();

    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    virtual void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr);

    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const;


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params);


    //!
    //! \brief Event to fire when an external module has been added
    //! \param resourceName Name of resource (module) added
    //! \param ID ID of module
    //!
    void ExternalModuleAdded(const char* ResourceName, int ID);

    void ExternalModuleRemoved(const char* ResourceName, int ID);

    std::string createLog(const int &systemID);

    virtual void TransmitMessage(const mace_message_t &msg, const OptionalParameter<MaceCore::ModuleCharacteristic> &target = OptionalParameter<MaceCore::ModuleCharacteristic>()) const;

    ///////////////////////////////////////////////////////////////////////////////////////
    /// The following are public virtual functions imposed from the Heartbeat Controller
    /// Interface via callback functionality.
    ///////////////////////////////////////////////////////////////////////////////////////
    void cbiHeartbeatController_transmitCommand(const mace_heartbeat_t &heartbeat);


    void ReceivedMission(const MissionItem::MissionList &list);
    Controllers::DataItem<MissionKey, MissionList>::FetchKeyReturn FetchMissionFromKey(const OptionalParameter<MissionKey> &key);
    Controllers::DataItem<MissionKey, MissionList>::FetchModuleReturn FetchAllMissionFromModule(const OptionalParameter<MaceCore::ModuleCharacteristic> &module);


    void ReceivedHome(const CommandItem::SpatialHome &home);
    Controllers::DataItem<MaceCore::ModuleCharacteristic, CommandItem::SpatialHome>::FetchKeyReturn FetchHomeFromKey(const OptionalParameter<MaceCore::ModuleCharacteristic> &key);
    Controllers::DataItem<MaceCore::ModuleCharacteristic, CommandItem::SpatialHome>::FetchModuleReturn FetchAllHomeFromModule(const OptionalParameter<MaceCore::ModuleCharacteristic> &module);

    void ReceivedCommand(const MaceCore::ModuleCharacteristic &sender, const std::shared_ptr<AbstractCommandItem> &command);


    bool isExternalLinkAirborne() const
    {
        return airborneInstance;
    }

    void ParseForData(const mace_message_t* message);


    void PublishVehicleData(const int &systemID, const std::shared_ptr<Data::ITopicComponentDataObject> &component);


    //!
    //! \brief New Mavlink message received over a link
    //! \param linkName Name of link message received over
    //! \param msg Message received
    //!
    virtual void MACEMessage(const std::string &linkName, const mace_message_t &msg);

    //!
    //! \brief VehicleHeartbeatInfo
    //! \param linkName
    //! \param systemID
    //! \param heartbeatMSG
    //!
    void HeartbeatInfo(const int &systemID, const mace_heartbeat_t &heartbeatMSG);


    //!
    //! \brief New non-spooled topic given
    //!
    //! NonSpooled topics send their data immediatly.
    //! \param topicName Name of stopic
    //! \param sender Module that sent topic
    //! \param data Data for topic
    //! \param target Target module (or broadcasted)
    //!
    virtual void NewTopicGiven(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const MaceCore::TopicDatagram &data, const OptionalParameter<MaceCore::ModuleCharacteristic> &target);


    //!
    //! \brief New Spooled topic given
    //!
    //! Spooled topics are stored on the core's datafusion.
    //! This method is used to notify other modules that there exists new data for the given components on the given module.
    //! \param topicName Name of topic given
    //! \param sender Module that sent topic
    //! \param componentsUpdated Components in topic that where updated
    //! \param target Target moudle (or broadcast)
    //!
    virtual void NewTopicAvailable(const std::string &topicName, const MaceCore::ModuleCharacteristic &sender, const std::vector<std::string> &componentsUpdated, const OptionalParameter<MaceCore::ModuleCharacteristic> &target = OptionalParameter<MaceCore::ModuleCharacteristic>());



    ///////////////////////////////////////////////////////////////////////////////////////
    /// The following are public virtual functions imposed from IModuleCommandExternalLink
    /// via the base listener class.
    ///////////////////////////////////////////////////////////////////////////////////////

public:
    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
    /// command and action sequence that accompanies the vheicle. Expect an
    /// acknowledgement or an event to take place when calling these items.
    ////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Request_FullDataSync
    //! \param targetSystem
    //!
    virtual void Request_FullDataSync(const int &targetSystem, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>());

    //!
    //! \brief Command_ChangeVehicleArm
    //! \param vehicleArm
    //!
    virtual void Command_SystemArm(const CommandItem::ActionArm &systemArm, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);

    //!
    //! \brief Command_ChangeVehicleOperationalMode
    //! \param vehicleMode
    //!
    virtual void Command_ChangeSystemMode(const CommandItem::ActionChangeMode &vehicleMode, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);

    //!
    //! \brief Command_RequestVehicleTakeoff
    //! \param vehicleTakeoff
    //!
    virtual void Command_VehicleTakeoff(const CommandItem::SpatialTakeoff &vehicleTakeoff, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);

    //!
    //! \brief Command_Land
    //! \param command
    //!
    virtual void Command_Land(const CommandItem::SpatialLand &vehicleLand, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);

    //!
    //! \brief Command_ReturnToLaunch
    //! \param command
    //!
    virtual void Command_ReturnToLaunch(const CommandItem::SpatialRTL &vehicleRTL, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);


    //!
    //! \brief Command_MissionState
    //! \param command
    //!
    virtual void Command_MissionState(const CommandItem::ActionMissionCommand &command, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);

    //!
    //! \brief Command_IssueGeneralCommand
    //! \param command
    //!
    virtual void Command_IssueGeneralCommand(const std::shared_ptr<CommandItem::AbstractCommandItem> &command);

    //!
    //! \brief Command_EmitHeartbeat
    //! \param heartbeat
    //!
    virtual void Command_EmitHeartbeat(const CommandItem::SpatialTakeoff &heartbeat);

    /////////////////////////////////////////////////////////////////////////
    /// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
    /// This functionality may be pertinent for vehicles not containing a
    /// direct MACE hardware module.
    /////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Command_UploadMission This function allows for a MACE instance to set
    //! a mission queue of a remote MACE instance. This is the only time this should be
    //! called. Missions at this point should merely be in a state of proposed as
    //! the it will be up to the remote instance to confirm receipt and action. No changes
    //! should be made with this associated list state until such event takes place.
    //! \param missionList The mission desired to be transmitted to the remote instance.
    //!
    virtual void Command_UploadMission(const MissionItem::MissionList &missionList);

    virtual void Command_GetCurrentMission(const int &targetSystem);

    virtual void Command_SetCurrentMission(const MissionItem::MissionKey &key);
    virtual void Command_GetMission(const MissionItem::MissionKey &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>());
    virtual void Command_ClearCurrentMission(const int &targetSystem);


    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL AUTO COMMANDS: This is implying for auto mode of the vehicle.
    /// This functionality is pertinent for vehicles that may contain a
    /// MACE HW module, or, vehicles that have timely or ever updating changes.
    ////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief SetCurrentGuidedQueue
    //! \param missionList
    //!
    virtual void Command_GetOnboardAuto(const int &targetSystem);

    //!
    //! \brief RequestCurrentGuidedQueue
    //! \param vehicleID
    //!
    virtual void Command_ClearOnboardAuto (const int &targetSystem);


    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL GUIDED COMMANDS: This is implying for guided mode of the vehicle.
    /// This functionality is pertinent for vehicles that may contain a
    /// MACE HW module, or, vehicles that have timely or ever updating changes.
    ////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief SetCurrentGuidedQueue
    //! \param missionList
    //!
    virtual void Command_GetOnboardGuided(const int &targetSystem);

    //!
    //! \brief RequestCurrentGuidedQueue
    //! \param vehicleID
    //!
    virtual void Command_ClearOnboardGuided (const int &targetSystem);


    /////////////////////////////////////////////////////////////////////////////
    /// GENERAL HOME EVENTS: These events are related to establishing or setting
    /// a home position. It should be recognized that the first mission item in a
    /// mission queue should prepend this position. Just the way ardupilot works.
    /////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief Command_GetHomePosition
    //! \param vehicleID
    //!
    virtual void Command_GetHomePosition (const int &vehicleID, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>());

    //!
    //! \brief Command_SetHomePosition
    //! \param vehicleHome
    //!
    virtual void Command_SetHomePosition(const CommandItem::SpatialHome &systemHome, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>());

    ///////////////////////////////////////////////////////////////////////////////////////
    /// The following are public virtual functions imposed from IModuleCommandExternalLink.
    ///////////////////////////////////////////////////////////////////////////////////////

    virtual void NewlyAvailableOnboardMission(const MissionItem::MissionKey &key, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender = OptionalParameter<MaceCore::ModuleCharacteristic>());
    virtual void NewlyAvailableHomePosition(const CommandItem::SpatialHome &home, const OptionalParameter<MaceCore::ModuleCharacteristic> &sender);
    virtual void NewlyAvailableMissionExeState(const MissionItem::MissionKey &missionKey);
    virtual void NewlyAvailableModule(const MaceCore::ModuleCharacteristic &module);
    virtual void ReceivedMissionACK(const MissionItem::MissionACK &ack);

private:
    ExternalLink::HeartbeatController_ExternalLink *m_HeartbeatController;

    PointerCollection<
        Controllers::CommandTakeoff<mace_message_t>,
        Controllers::CommandLand<mace_message_t>,
        Controllers::CommandARM<mace_message_t>,
        Controllers::CommandRTL<mace_message_t>,
        Controllers::CommandMissionItem<mace_message_t>,
        Controllers::ControllerSystemMode<mace_message_t>,
        Controllers::ControllerHome<mace_message_t>,
        Controllers::ControllerMission<mace_message_t>
    > m_Controllers;

private:
    bool airborneInstance;
    //!
    //! \brief associatedSystemID This is the identifier that is transmitting the data as a representative of.
    //! In the case of an airborne instance it will be assigned to the value of the heartbeat message recieved
    //! over the internal mace network. Thus, outbound transmissions will be relevant to the request of the
    //! system to which to it attached. It will be defaulted to match the GCS ID.
    //!
    int associatedSystemID;
    std::map<int,int> systemIDMap;

    std::shared_ptr<spdlog::logger> mLog;

    MaceCore::SpooledTopic<DATA_GENERIC_VEHICLE_ITEM_TOPICS, DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;
    MaceCore::SpooledTopic<DATA_MISSION_GENERIC_TOPICS> m_MissionDataTopic;

    BaseTopic::VehicleTopics m_VehicleTopics;
};

#endif // MODULE_EXTERNAL_LINK_H
