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
    ModuleVehicleMAVLINK<DATA_VEHICLE_ARDUPILOT_TYPES>()
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
//    if(topicName == m_CommandVehicleTopic.Name())
//    {
//        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_CommandVehicleTopic.Name(), senderID);
//        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
//            if(componentsUpdated.at(i) == DataVehicleCommands::ActionCommandTopic::Name()) {
//                std::shared_ptr<DataVehicleCommands::ActionCommandTopic> component = std::make_shared<DataVehicleCommands::ActionCommandTopic>();
//                m_CommandVehicleTopic.GetComponent(component, read_topicDatagram);
//                switch(component->getActionItemType())
//                {
//                case(DataVehicleCommands::ActionCommandTypes::CHANGE_MODE):
//                {
//                    //should find a better way to do this
////                    if(m_ArduPilotMAVLINKParser.heartbeatUpdated())
////                    {
////                        DataVehicleCommands::CommandVehicleMode* cmdMode = (DataVehicleCommands::CommandVehicleMode*)component->getActionItem().get();
////                        int newMode = m_ArduPilotMAVLINKParser.getFlightModeFromString(cmdMode->getRequestMode());
////                        uint8_t chan = m_LinkMarshaler->GetProtocolChannel("link1");
////                        mavlink_message_t msg;
////                        mavlink_msg_set_mode_pack_chan(255,190,chan,&msg,1,MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,newMode);
////                        m_LinkMarshaler->SendMessage<mavlink_message_t>("link1", msg);
////                    }
//                    break;
//                }
//                case(DataVehicleCommands::ActionCommandTypes::ARM):
//                {
//                    break;
//                }
//                }
//            }
//        }
//    } else if(topicName == m_CommandVehicleMissionList.Name())
//    {
//        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_CommandVehicleMissionList.Name(), senderID);
//        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
//            if(componentsUpdated.at(i) == DataVehicleCommands::VehicleMissionList::Name()) {
//                std::shared_ptr<DataVehicleCommands::VehicleMissionList> component = std::make_shared<DataVehicleCommands::VehicleMissionList>();
//                std::cout<<"The before"<<std::endl;
//                m_CommandVehicleMissionList.GetComponent(component, read_topicDatagram);
//                std::cout<<"The after"<<std::endl;
//            }
//        }
//    }
}
