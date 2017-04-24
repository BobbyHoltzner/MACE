#ifndef MODULE_EXTERNAL_LINK_H
#define MODULE_EXTERNAL_LINK_H

#include <iostream>

#include "module_external_link_global.h"
#include <chrono>

#include "mavlink.h"

#include "common/common.h"

#include "data_comms/data_external_comms.h"
#include "data_comms/MACE_to_COMMS/state_mace_to_comms.h"
#include "data_comms/MACE_to_COMMS/mission_mace_to_comms.h"

#include "commsMAVLINK/comms_mavlink.h"

#include "mace_core/i_module_topic_events.h"
#include "mace_core/i_module_command_external_link.h"
#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data_vehicle_sensors/components.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_item_topic/data_generic_item_topic_components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

class MODULE_EXTERNAL_LINKSHARED_EXPORT ModuleExternalLink :
        public MaceCore::IModuleCommandExternalLink,
        public CommsMAVLINK
{

public:

    ModuleExternalLink();

    void ParseForData(const mavlink_message_t* message);

    void ParseCommsCommand(const mavlink_command_long_t* message);

    //!
    //! \brief New Mavlink message received over a link
    //! \param linkName Name of link message received over
    //! \param msg Message received
    //!
    virtual void MavlinkMessage(const std::string &linkName, const mavlink_message_t &msg);
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
    /// The following are public virtual functions imposed from IModuleCommandExternalLink.
    ///////////////////////////////////////////////////////////////////////////////////////

public:
    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
    /// command and action sequence that accompanies the vheicle. Expect an
    /// acknowledgement or an event to take place when calling these items.
    ////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief ChangeVehicleArm
    //! \param vehicleArm
    //!
    virtual void ChangeVehicleArm(const MissionItem::ActionArm &vehicleArm);

    //!
    //! \brief ChangeVehicleOperationalMode
    //! \param vehicleMode
    //!
    virtual void ChangeVehicleOperationalMode(const MissionItem::ActionChangeMode &vehicleMode);

    //!
    //! \brief RequestVehicleTakeoff
    //! \param vehicleTakeoff
    //!
    virtual void RequestVehicleTakeoff(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &vehicleTakeoff);


    /////////////////////////////////////////////////////////////////////////
    /// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
    /// This functionality may be pertinent for vehicles not containing a
    /// direct MACE hardware module.
    /////////////////////////////////////////////////////////////////////////

    //!
    //! \brief SetCurrentMissionQueue This function allows for a MACE instance to set
    //! a mission queue of a remote MACE instance. This is the only time this should be
    //! called. Missions at this point should merely be in a state of proposed as
    //! the it will be up to the remote instance to confirm receipt and action. No changes
    //! should be made with this associated list state until such event takes place.
    //! \param missionList The mission desired to be transmitted to the remote instance.
    //!
    virtual void SetMissionQueue(const MissionItem::MissionList &missionList);

    //!
    //! \brief RequestCurrentMissionQueue
    //! \param vehicleID
    //!
    virtual void GetMissionQueue (const int &targetSystem);

    //!
    //! \brief RequestClearMissionQueue
    //! \param vehicleID
    //!
    virtual void ClearMissionQueue (const int &targetSystem);


    ////////////////////////////////////////////////////////////////////////////
    /// GENERAL GUIDED EVENTS: This is implying for guided mode of the vehicle.
    /// This functionality is pertinent for vehicles that may contain a
    /// MACE HW module, or, vehicles that have timely or ever updating changes.
    ////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief SetCurrentGuidedQueue
    //! \param missionList
    //!
    virtual void SetCurrentGuidedQueue(const MissionItem::MissionList &missionList);

    //!
    //! \brief RequestCurrentGuidedQueue
    //! \param vehicleID
    //!
    virtual void RequestCurrentGuidedQueue (const int &vehicleID);

    //!
    //! \brief RequestClearGuidedQueue
    //! \param vehicleID
    //!
    virtual void RequestClearGuidedQueue (const int &vehicleID);


    /////////////////////////////////////////////////////////////////////////////
    /// GENERAL HOME EVENTS: These events are related to establishing or setting
    /// a home position. It should be recognized that the first mission item in a
    /// mission queue should prepend this position. Just the way ardupilot works.
    /////////////////////////////////////////////////////////////////////////////

    //!
    //! \brief RequestVehicleHomePosition
    //! \param vehicleID
    //!
    virtual void RequestVehicleHomePosition (const int &vehicleID);

    //!
    //! \brief SetVehicleHomePosition
    //! \param vehicleHome
    //!
    virtual void SetVehicleHomePosition(const MissionItem::SpatialHome &vehicleHome);

private:
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
