#ifndef MODULE_EXTERNAL_LINK_H
#define MODULE_EXTERNAL_LINK_H

#include <iostream>
#include <stdint.h>

#include "module_external_link_global.h"
#include <chrono>

#include "mace.h"

#include "common/common.h"

#include "data_comms/data_external_comms.h"
#include "data_comms/MACE_to_COMMS/state_mace_to_comms.h"
#include "data_comms/MACE_to_COMMS/mission_mace_to_comms.h"

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

class MODULE_EXTERNAL_LINKSHARED_EXPORT ModuleExternalLink :
        public MaceCore::IModuleCommandExternalLink,
        public CommsMACEHelper
{

public:

    ModuleExternalLink();

    void ParseForData(const mace_message_t* message);

    void ParseCommsCommand(const mace_command_long_t* message);

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
    virtual void MACEHeartbeatInfo(const std::string &linkName, const int &systemID, const mace_heartbeat_t &heartbeatMSG);

    //!
    //! \brief MACESyncMessage
    //! \param linkName
    //! \param systemID
    //! \param syncMSG
    //!
    virtual void MACESyncMessage(const std::string &linkName, const int &systemID, const mace_vehicle_sync_t &syncMSG);

    //!
    //! \brief VehicleCommandMACEACK
    //! \param linkName
    //! \param systemID
    //! \param cmdACK
    //!
    virtual void MACECommandACK(const std::string &linkName, const int &systemID, const mace_command_ack_t &cmdACK);

    //!
    //! \brief NewTopic
    //! \param topicName
    //! \param senderID
    //! \param componentsUpdated
    //!
    virtual void NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated);

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
    //! \brief Command_ChangeVehicleArm
    //! \param vehicleArm
    //!
    virtual void Command_SystemArm(const CommandItem::ActionArm &vehicleArm);

    //!
    //! \brief Command_ChangeVehicleOperationalMode
    //! \param vehicleMode
    //!
    virtual void Command_ChangeSystemMode(const CommandItem::ActionChangeMode &vehicleMode);

    //!
    //! \brief Command_RequestVehicleTakeoff
    //! \param vehicleTakeoff
    //!
    virtual void Command_VehicleTakeoff(const CommandItem::SpatialTakeoff<DataState::StateGlobalPosition> &vehicleTakeoff);

    //!
    //! \brief Command_Land
    //! \param command
    //!
    virtual void Command_Land(const CommandItem::SpatialLand<DataState::StateGlobalPosition> &command);

    //!
    //! \brief Command_ReturnToLaunch
    //! \param command
    //!
    virtual void Command_ReturnToLaunch(const CommandItem::SpatialRTL &command);

    //!
    //! \brief Command_IssueGeneralCommand
    //! \param command
    //!
    virtual void Command_IssueGeneralCommand(const std::shared_ptr<CommandItem::AbstractCommandItem> &command);

    //!
    //! \brief Command_EmitHeartbeat
    //! \param heartbeat
    //!
    virtual void Command_EmitHeartbeat(const CommandItem::SpatialTakeoff<DataState::StateGlobalPosition> &heartbeat);

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

    virtual void Command_SetCurrentMission(const Data::MissionKey &key);
    virtual void Command_GetMission(const Data::MissionKey &key);
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
    virtual void Command_GetHomePosition (const int &vehicleID);

    //!
    //! \brief Command_SetHomePosition
    //! \param vehicleHome
    //!
    virtual void Command_SetHomePosition(const CommandItem::SpatialHome &vehicleHome);

    ///////////////////////////////////////////////////////////////////////////////////////
    /// The following are public virtual functions imposed from IModuleCommandExternalLink.
    ///////////////////////////////////////////////////////////////////////////////////////

    virtual void NewlyAvailableOnboardMission(const Data::MissionKey &key);
    virtual void NewlyAvailableHomePosition(const CommandItem::SpatialHome &home);

private:
    bool airborneInstance;
    bool firstHearbeat;
    //!
    //! \brief associatedSystemID This is the identifier that is transmitting the data as a representative of.
    //! In the case of an airborne instance it will be assigned to the value of the heartbeat message recieved
    //! over the internal mace network. Thus, outbound transmissions will be relevant to the request of the
    //! system to which to it attached. It will be defaulted to match the GCS ID.
    //!
    int associatedSystemID;
    std::map<int,int> systemIDMap;

    Data::TopicDataObjectCollection<DATA_GENERIC_VEHICLE_ITEM_TOPICS, DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;
    Data::TopicDataObjectCollection<DATA_MISSION_GENERIC_TOPICS> m_MissionDataTopic;
};

#endif // MODULE_EXTERNAL_LINK_H
