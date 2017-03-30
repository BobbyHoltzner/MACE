#ifndef MODULE_VEHICLE_ARDUPILOT_H
#define MODULE_VEHICLE_ARDUPILOT_H

#include <map>

#include <mavlink.h>

#include "data/timer.h"

#include "module_vehicle_ardupilot_global.h"
#include "module_vehicle_MAVLINK/module_vehicle_mavlink.h"

#include "data_vehicle_ardupilot/data_vehicle_ardupilot.h"
#include "data_vehicle_ardupilot/ardupilot_to_mace.h"
#include "data_vehicle_ardupilot/mace_to_ardupilot.h"
#include "data_vehicle_ardupilot/components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

//__________________
#include "data_vehicle_mavlink/MACE_to_MAVLINK/command_mace_to_mavlink.h"

class MODULE_VEHICLE_ARDUPILOTSHARED_EXPORT ModuleVehicleArdupilot : public ModuleVehicleMAVLINK<DATA_VEHICLE_ARDUPILOT_TYPES>
{
enum ArdupilotMissionMode{
    NONE,
    REQUESTING,
    TRANSMITTING
};

public:
    ModuleVehicleArdupilot();

    bool ParseMAVLINKMissionMessage(const std::string &linkName, const mavlink_message_t *message);

    void MissionAcknowledgement(const MAV_MISSION_RESULT &missionResult, const bool &publishResult);


public:
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

    ////////////////////////////////////////////////////////////////////////////////
    /// The following are public virtual functions imposed from IModuleCommandVehicle.
    ////////////////////////////////////////////////////////////////////////////////
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
    //! \brief SetCurrentMissionQueue
    //! \param missionList
    //!
    virtual void SetMissionQueue(const MissionItem::MissionList &missionList);

    //!
    //! \brief RequestCurrentMissionQueue
    //! \param vehicleID
    //!
    virtual void GetMissionQueue (const Data::SystemDescription &targetSystem);

    //!
    //! \brief RequestClearMissionQueue
    //! \param vehicleID
    //!
    virtual void ClearMissionQueue (const Data::SystemDescription &targetSystem);


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
    bool firstHeartbeat;

    Timer t;

private:
    Data::TopicDataObjectCollection<DATA_MISSION_GENERIC_TOPICS> m_VehicleMission;

    std::map<int,DataArdupilot::DataVehicleArdupilot*> m_ArduPilotData;


    std::shared_ptr<MissionTopic::MissionItemCurrentTopic> m_CurrentMissionItem;

};

#endif // MODULE_VEHICLE_ARDUPILOT_H
