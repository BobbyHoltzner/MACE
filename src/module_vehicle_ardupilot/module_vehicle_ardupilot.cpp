#include "module_vehicle_ardupilot.h"

#include "data_vehicle_ardupilot/mavlink_parser_ardupilot.h"

ModuleVehicleArdupilot::ModuleVehicleArdupilot() :
    ModuleVehicleMAVLINK<>()
{
}


//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleVehicleArdupilot::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
}


//!
//! \brief New Mavlink message received over a link
//! \param linkName Name of link message received over
//! \param msg Message received
//!
void ModuleVehicleArdupilot::MavlinkMessage(const std::string &linkName, const mavlink_message_t &message)
{
    ModuleVehicleMAVLINK<>::MavlinkMessage(linkName, message);

    int sendersID = (int)message.sysid;
    int messageID = (int)message.msgid;


    //generate topic datagram from given mavlink message
    std::shared_ptr<MaceCore::TopicDatagram> topicDatagram = m_ArduPilotMAVLINKParser.Parse<VehicleDataTopicType>(&message, &m_VehicleDataTopic);


    //if we generated something then notify any attached MaceCore::IModuleTopicEvents listeners
    if(topicDatagram != NULL)
    {
        //notify of new topic datagram
        NotifyListeners([&, topicDatagram](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(m_VehicleDataTopic.Name(), 1, MaceCore::TIME(), *topicDatagram);
        });
    }

}


void ModuleVehicleArdupilot::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{

}
