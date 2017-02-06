#include "module_vehicle_ardupilot.h"
void ModuleVehicleArdupilot::ChangeVehicleArm(const MissionItem::ActionArm &vehicleArm)
{
    MissionItem::ActionArm* armMsg = new MissionItem::ActionArm(vehicleArm);
    uint8_t chan = m_LinkMarshaler->GetProtocolChannel("link1");
    mavlink_message_t msg = m_ArduPilotMAVLINKParser.at(vehicleArm.getVehicleID())->generateArdupilotMessage(armMsg,chan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>("link1", msg);

}

void ModuleVehicleArdupilot::ChangeVehicleOperationalMode(const MissionItem::ActionChangeMode &vehicleMode)
{
    MissionItem::ActionChangeMode* armMsg = new MissionItem::ActionChangeMode(vehicleMode);
    uint8_t chan = m_LinkMarshaler->GetProtocolChannel("link1");
    mavlink_message_t msg = m_ArduPilotMAVLINKParser.at(vehicleMode.getVehicleID())->generateArdupilotMessage(armMsg,chan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>("link1", msg);
}


ModuleVehicleArdupilot::ModuleVehicleArdupilot() :
    ModuleVehicleMAVLINK<DATA_VEHICLE_ARDUPILOT_TYPES>(), m_VehicleMission("externalMission")
{
}


//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleVehicleArdupilot::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
    ptr->Subscribe(this, m_VehicleMission.Name());
}


//!
//! \brief New Mavlink message received over a link
//! \param linkName Name of link message received over
//! \param msg Message received
//!
void ModuleVehicleArdupilot::MavlinkMessage(const std::string &linkName, const mavlink_message_t &message)
{
    ModuleVehicleMAVLINK<DATA_VEHICLE_ARDUPILOT_TYPES>::MavlinkMessage(linkName, message);

    DataVehicleArdupilot::MAVLINKParserArduPilot* tmpParser;
    int newSystemID = message.sysid;
    try{
        tmpParser = m_ArduPilotMAVLINKParser.at(newSystemID);
    }catch(const std::out_of_range &oor)
    {
        std::cout<<"This vehicle parser was not currently in the map. Going to add one."<<std::endl;
        tmpParser = new DataVehicleArdupilot::MAVLINKParserArduPilot();
        m_ArduPilotMAVLINKParser.insert({newSystemID,tmpParser});
        ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
            ptr->NewConstructedVehicle(this, newSystemID);
        });
    }

    //generate topic datagram from given mavlink message
    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> components = tmpParser->Parse(&message);

    //procede to send components only if there is 1 or more
    if(components.size() > 0)
    {

        //construct datagram
        MaceCore::TopicDatagram topicDatagram;
        for(size_t i = 0 ; i < components.size() ; i++)
        {
            m_VehicleDataTopic.SetComponent(components.at(i), topicDatagram);
        }

        //notify listneres of topic
        ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(m_VehicleDataTopic.Name(), 1, MaceCore::TIME(), topicDatagram);
        });
    }
}


void ModuleVehicleArdupilot::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    //MissionTopic::MissionType newType = MissionTopic::MissionType::ACTION;
    if(topicName == m_VehicleMission.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleMission.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++){
            if(componentsUpdated.at(i) == MissionTopic::MissionItemTopic::Name()) {
                std::shared_ptr<MissionTopic::MissionItemTopic> component = std::make_shared<MissionTopic::MissionItemTopic>();
                m_VehicleMission.GetComponent(component, read_topicDatagram);
            }
            else if(componentsUpdated.at(i) == MissionTopic::MissionListTopic::Name()){
                std::shared_ptr<MissionTopic::MissionListTopic> component = std::make_shared<MissionTopic::MissionListTopic>();
                m_VehicleMission.GetComponent(component, read_topicDatagram);
            }
        }
    }
}
