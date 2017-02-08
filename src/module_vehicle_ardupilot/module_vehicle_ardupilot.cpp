#include "module_vehicle_ardupilot.h"

ModuleVehicleArdupilot::ModuleVehicleArdupilot() :
    ModuleVehicleMAVLINK<DATA_VEHICLE_ARDUPILOT_TYPES>(), m_VehicleMission("externalMission"),
    msgDumpCounter(0),missionMode(NONE),missionItemIndex(0),missionItemsAvailable(0)
{

}

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

    bool wasMissionMSG = ParseMAVLINKMissionMessage(linkName, &message);

    if(wasMissionMSG == false){
        //generate topic datagram from given mavlink message
        std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> components = tmpParser->ParseForVehicleData(&message);
        //procede to send components only if there is 1 or more
        if(components.size() > 0)
        {
            //construct datagram
            MaceCore::TopicDatagram topicDatagram;
            for(size_t i = 0 ; i < components.size() ; i++)
            {
                m_VehicleDataTopic.SetComponent(components.at(i), topicDatagram);
                //notify listneres of topic
                ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                    ptr->NewTopicDataValues(m_VehicleDataTopic.Name(), 1, MaceCore::TIME(), topicDatagram);
                });
            }
        } //if there is information available
    }
}

void ModuleVehicleArdupilot::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    uint8_t chan = m_LinkMarshaler->GetProtocolChannel("link1");

    if(topicName == m_VehicleMission.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleMission.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++){
            if(componentsUpdated.at(i) == MissionTopic::MissionItemTopic::Name()) {
                std::shared_ptr<MissionTopic::MissionItemTopic> component = std::make_shared<MissionTopic::MissionItemTopic>();
                m_VehicleMission.GetComponent(component, read_topicDatagram);
                //mavlink_message_t msg = m_ArduPilotMAVLINKParser.at(component->getVehicleID())->generateArdupilotMessage(component->getMissionItem(),chan);
            }
            else if(componentsUpdated.at(i) == MissionTopic::MissionListTopic::Name()){
                std::shared_ptr<MissionTopic::MissionListTopic> component = std::make_shared<MissionTopic::MissionListTopic>();
                m_VehicleMission.GetComponent(component, read_topicDatagram);
                if(component->getMissionType() == MissionTopic::MissionType::MISSION){
                    int vehicleID = component->getVehicleID();
                    m_ProposedMissionQueue[vehicleID] = *component->getMissionList();
                    mavlink_message_t msg;
//                    mavlink_msg_mission_count_pack_chan(255,190,chan,&msg,vehicleID,0,m_ProposedMissionQueue.at(vehicleID).getQueueSize());
//                    m_LinkMarshaler->SendMessage<mavlink_message_t>("link1", msg);
                    missionMode = REQUESTING;
                    mavlink_msg_mission_request_list_pack_chan(255,190,chan,&msg,1,0);
                    m_LinkMarshaler->SendMessage<mavlink_message_t>("link1", msg);

                }else if(component->getMissionType() == MissionTopic::MissionType::GUIDED){

                }else if(component->getMissionType() == MissionTopic::MissionType::ACTION){

                }
                //m_ArduPilotMAVLINKParser.at(component->getVehicleID())->generateArdupilotMessage(component->getMissionItem(),chan);
            }
        }
    }
}



