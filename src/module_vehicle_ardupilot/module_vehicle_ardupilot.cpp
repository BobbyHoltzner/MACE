#include "module_vehicle_ardupilot.h"

#include "data_vehicle_ardupilot/mavlink_parser_ardupilot.h"

ModuleVehicleArdupilot::ModuleVehicleArdupilot() :
    ModuleVehicleMAVLINK<>()
{
}


//!
//! \brief New Mavlink message received over a link
//! \param linkName Name of link message received over
//! \param msg Message received
//!
void ModuleVehicleArdupilot::MavlinkMessage(const std::string &linkName, const mavlink_message_t &message) const
{

    int sendersID = (int)message.sysid;
    int messageID = (int)message.msgid;

    //get maping of all vehicle data components
    MaceCore::TopicDatagram topicDatagram = m_ArduPilotMAVLINKParser.Parse<VehicleDataTopicType>(&message, &m_VehicleDataTopic);

    //notify of new topic datagram
    NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
        ptr->NewTopicDataValues(m_VehicleDataTopic.Name(), 1, MaceCore::TIME(), topicDatagram);
    });

}
