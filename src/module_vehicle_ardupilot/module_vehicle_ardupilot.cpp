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
void ModuleVehicleArdupilot::MavlinkMessage(const std::string &linkName, const mavlink_message_t &message)
{
    //Base::MavlinkMessage(linkName, message);

    int sendersID = (int)message.sysid;
    int messageID = (int)message.msgid;

    //get maping of all vehicle data components
    MaceCore::TopicDatagram topicDatagram = m_ArduPilotMAVLINKParser.Parse<VehicleDataTopicType>(&message, &m_VehicleDataTopic);

    //notify of new topic datagram
    NotifyListeners([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(m_VehicleDataTopic.Name(), 1, MaceCore::TIME(), topicDatagram);
    });

}


void ModuleVehicleArdupilot::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    //example read
    /*
    MaceCore::TopicDatagram read_topicDatagram = ModuleVehicleGeneric<VehicleTopicAdditionalComponents...>::getDataObject()->GetCurrentTopicDatagram(VehicleDataTopicPtr->Name(), 1);
    std::shared_ptr<DataVehicleGeneric::GlobalPosition> position_component = ((Data::TopicDataObjectCollection<DataVehicleGeneric::GlobalPosition>)*VehicleDataTopicPtr).GetComponent(read_topicDatagram);
    if(position_component != NULL) {
        std::cout << position_component->latitude << " " << position_component->longitude << std::endl;
    }
    */
}
