#ifndef MODULE_VEHICLE_ARDUPILOT_H
#define MODULE_VEHICLE_ARDUPILOT_H

#include <map>

#include <mavlink.h>

#include "data/timer.h"

#include "module_vehicle_ardupilot_global.h"
#include "module_vehicle_MAVLINK/module_vehicle_mavlink.h"
#include "data_vehicle_ardupilot/mavlink_parser_ardupilot.h"
#include "data_vehicle_ardupilot/components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"


#include "mission_parser_vehicle_ardupilot.h"

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

    virtual void NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated);

    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    virtual void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr);

public:
    virtual void ChangeVehicleArm(const MissionItem::ActionArm &vehicleArm);

    virtual void ChangeVehicleOperationalMode(const MissionItem::ActionChangeMode &vehicleMode);

    virtual void RequestVehicleHomePosition (const int &vehicleID);

    virtual void RequestCurrentMissionQueue (const int &vehicleID);

    virtual void RequestClearMissionQueue (const int &vehicleID);

    virtual void RequestCurrentGuidedQueue (const int &vehicleID);

    virtual void RequestClearGuidedQueue (const int &vehicleID);

private:
    std::map<int,MissionItem::MissionList> m_CurrentMissionQueue;
    std::map<int,MissionItem::MissionList> m_ProposedMissionQueue;

    std::map<int,MissionItem::MissionList> m_CurrentGuidedQueue;
    std::map<int,MissionItem::MissionList> m_ProposedGuidedQueue;

private:
    ArdupilotMissionMode missionMode;
    int missionMSGCounter;
    int missionItemIndex;
    int missionItemsAvailable;

    Timer t;

private:
    Data::TopicDataObjectCollection<DATA_MISSION_GENERIC_TOPICS> m_VehicleMission;

    std::map<int,DataVehicleArdupilot::MAVLINKParserArduPilot*> m_ArduPilotMAVLINKParser;

};

#endif // MODULE_VEHICLE_ARDUPILOT_H
