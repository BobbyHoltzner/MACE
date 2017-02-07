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

    bool wasMissionMSG = ParseForMissionMessage(linkName, &message);

    if( wasMissionMSG == false){
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



bool ModuleVehicleArdupilot::ParseForMissionMessage(const std::string &linkName, const mavlink_message_t* message)
{
    bool parsedMissionMSG = true;
    int sysID = message->sysid;
    int compID = message->compid;
    uint8_t chan = m_LinkMarshaler->GetProtocolChannel(linkName);

    switch ((int)message->msgid) {

    case MAVLINK_MSG_ID_MISSION_REQUEST:
    {
        //This is message definition 40
        //Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. http://qgroundcontrol.org/mavlink/waypoint_protocol
        mavlink_mission_request_t decodedMSG;
        mavlink_msg_mission_request_decode(message,&decodedMSG);
        std::cout<<"The aircraft is requesting item number: "<<decodedMSG.seq<<std::endl;
        std::shared_ptr<MissionItem::AbstractMissionItem> missionItem = m_ProposedMissionQueue.at(sysID).getMissionItem(decodedMSG.seq);
        mavlink_message_t msg = MissionParserArdupilot::generateMissionMessage(missionItem,decodedMSG.seq,compID,chan);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(linkName, msg);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
    {
        //This is message definition 41
        //Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between).
        mavlink_mission_set_current_t decodedMSG;
        mavlink_msg_mission_set_current_decode(message,&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_CURRENT:
    {
        //This is message definition 42
        //Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item.
        mavlink_mission_current_t decodedMSG;
        mavlink_msg_mission_current_decode(message,&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    {
        //This is message definition 43
        //Request the overall list of mission items from the system/component.
        mavlink_mission_request_list_t decodedMSG;
        mavlink_msg_mission_request_list_decode(message,&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_COUNT:
    {
        //This is message definition 44
        //This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of MISSIONs.
        mavlink_mission_count_t decodedMSG;
        mavlink_msg_mission_count_decode(message,&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
    {
        //This is message definition 45
        //Delete all mission items at once.
        mavlink_mission_clear_all_t decodedMSG;
        mavlink_msg_mission_clear_all_decode(message,&decodedMSG);
        break;
    }
    case MAVLINK_MSG_ID_MISSION_ACK:
    {
        //This is message definition 47
        //Ack message during MISSION handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
        mavlink_mission_ack_t decodedMSG;
        mavlink_msg_mission_ack_decode(message,&decodedMSG);
        std::cout<<"Mission Acknowledged?"<<std::endl;
        break;
    }

    default:
    {
        parsedMissionMSG = false;
    }
    }
    return parsedMissionMSG;
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
                    mavlink_msg_mission_count_pack_chan(255,190,chan,&msg,vehicleID,0,m_ProposedMissionQueue.at(vehicleID).getQueueSize());
                    m_LinkMarshaler->SendMessage<mavlink_message_t>("link1", msg);
                }else if(component->getMissionType() == MissionTopic::MissionType::GUIDED){

                }else if(component->getMissionType() == MissionTopic::MissionType::ACTION){

                }
                //m_ArduPilotMAVLINKParser.at(component->getVehicleID())->generateArdupilotMessage(component->getMissionItem(),chan);
            }
        }
    }
}


