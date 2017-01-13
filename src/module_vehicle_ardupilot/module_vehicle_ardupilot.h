#ifndef MODULE_VEHICLE_ARDUPILOT_H
#define MODULE_VEHICLE_ARDUPILOT_H

#include "module_vehicle_ardupilot_global.h"

#include "module_vehicle_MAVLINK/module_vehicle_mavlink.h"

#include "data_vehicle_ardupilot/mavlink_parser_ardupilot.h"

class MODULE_VEHICLE_ARDUPILOTSHARED_EXPORT ModuleVehicleArdupilot : public ModuleVehicleMAVLINK<>
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

private:

    DataVehicleArdupilot::MAVLINKParserArduPilot m_ArduPilotMAVLINKParser;
};

#endif // MODULE_VEHICLE_ARDUPILOT_H
