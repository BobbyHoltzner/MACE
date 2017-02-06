#ifndef MODULE_VEHICLE_ARDUPILOT_H
#define MODULE_VEHICLE_ARDUPILOT_H

#include <map>

#include <mavlink.h>

#include "module_vehicle_ardupilot_global.h"
#include "module_vehicle_MAVLINK/module_vehicle_mavlink.h"
#include "data_vehicle_ardupilot/mavlink_parser_ardupilot.h"
#include "data_vehicle_ardupilot/components.h"


#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"



class MODULE_VEHICLE_ARDUPILOTSHARED_EXPORT ModuleVehicleArdupilot : public ModuleVehicleMAVLINK<DATA_VEHICLE_ARDUPILOT_TYPES>
{

public:
    ModuleVehicleArdupilot();


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


public:
    void HandleCommandTopic();

    void HandleMissionTopic();

private:
    //Data::TopicDataObjectCollection<DATA_VEHICLE_ACTION_COMMAND_TYPES> m_CommandVehicleTopic;
    //Data::TopicDataObjectCollection<DATA_VEHICLE_MISSION_LIST> m_CommandVehicleMissionList;


private:
    //DataVehicleArdupilot::MAVLINKParserArduPilot m_ArduPilotMAVLINKParser;
    Data::TopicDataObjectCollection<DATA_MISSION_GENERIC_TOPICS> m_VehicleMission;

    std::map<int,DataVehicleArdupilot::MAVLINKParserArduPilot*> m_ArduPilotMAVLINKParser;

};

#endif // MODULE_VEHICLE_ARDUPILOT_H
