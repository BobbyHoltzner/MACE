#include "module_vehicle_ardupilot.h"

ModuleVehicleArdupilot::ModuleVehicleArdupilot() :
    ModuleVehicleMAVLINK<DATA_VEHICLE_ARDUPILOT_TYPES>(), m_VehicleMission("vehicleMission"),
    missionMSGCounter(0),missionMode(NONE),missionItemIndex(0),missionItemsAvailable(0),
    m_CurrentMissionItem(NULL)
{

}

////////////////////////////////////////////////////////////////////////////
/// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
/// command and action sequence that accompanies the vheicle. Expect an
/// acknowledgement or an event to take place when calling these items.
////////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::ChangeVehicleArm(const MissionItem::ActionArm &vehicleArm)
{
    mavlink_message_t msg = DataVehicleArdupilot::generateArmMessage(vehicleArm,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::ChangeVehicleOperationalMode(const MissionItem::ActionChangeMode &vehicleMode)
{
    MissionItem::ActionChangeMode* armMsg = new MissionItem::ActionChangeMode(vehicleMode);
    //FIX KENNY
    //mavlink_message_t msg = DataVehicleArdupilot::ArdupilotToMACEMission::generateChangeMode(vehicleArm,m_LinkChan);
    //mavlink_message_t msg = m_ArduPilotMAVLINKParser.at(vehicleMode.getVehicleID())->generateArdupilotMessage(armMsg,m_LinkChan);
    //m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

/////////////////////////////////////////////////////////////////////////////
/// GENERAL HOME EVENTS: These events are related to establishing or setting
/// a home position. It should be recognized that the first mission item in a
/// mission queue should prepend this position. Just the way ardupilot works.
/////////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::RequestVehicleHomePosition(const int &vehicleID)
{
    mavlink_message_t msg = DataVehicleArdupilot::generateGetHomePosition(vehicleID,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::SetVehicleHomePosition(const MissionItem::SpatialHome &vehicleHome)
{
    mavlink_message_t msg = DataVehicleArdupilot::generateSetHomePosition(vehicleHome,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

/////////////////////////////////////////////////////////////////////////
/// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
/// This functionality may be pertinent for vehicles not containing a
/// direct MACE hardware module.
/////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::SetCurrentMissionQueue(const MissionItem::MissionList &missionList)
{
    int vehicleID = missionList.getVehicleID();
    m_ProposedMissionQueue[vehicleID] = missionList;
    mavlink_message_t msg;
    int queueSize = m_ProposedMissionQueue.at(vehicleID).getQueueSize();
    mavlink_msg_mission_count_pack_chan(255,190,m_LinkChan,&msg,vehicleID,0,m_ProposedMissionQueue.at(vehicleID).getQueueSize());
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::RequestCurrentMissionQueue(const int &vehicleID)
{
    mavlink_message_t msg;
    missionMode = REQUESTING;
    mavlink_msg_mission_request_list_pack_chan(255,190,m_LinkChan,&msg,vehicleID,0);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::RequestClearMissionQueue(const int &vehicleID)
{
    //This is message number 45....
    //TODO: Do we get an acknowledgement from this?
    mavlink_message_t msg;
    mavlink_msg_mission_clear_all_pack_chan(255,190,m_LinkChan,&msg,vehicleID,0);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

////////////////////////////////////////////////////////////////////////////
/// GENERAL GUIDED EVENTS: This is implying for guided mode of the vehicle.
/// This functionality is pertinent for vehicles that may contain a
/// MACE HW module, or, vehicles that have timely or ever updating changes.
////////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::SetCurrentGuidedQueue(const MissionItem::MissionList &missionList)
{
    int vehicleID = missionList.getVehicleID();
}

void ModuleVehicleArdupilot::RequestCurrentGuidedQueue(const int &vehicleID)
{
    //This command is performed locally in the MACE instance.
}

void ModuleVehicleArdupilot::RequestClearGuidedQueue(const int &vehicleID)
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
        tmpParser = new DataVehicleArdupilot::MAVLINKParserArduPilot();
        m_ArduPilotMAVLINKParser.insert({newSystemID,tmpParser});

        MissionItem::MissionList newMissionList;
        m_CurrentMissionQueue.insert({newSystemID,newMissionList});
        m_ProposedMissionQueue.insert({newSystemID,newMissionList});

        m_CurrentGuidedQueue.insert({newSystemID,newMissionList});
        m_ProposedGuidedQueue.insert({newSystemID,newMissionList});

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
                if(component->getMissionType() == MissionTopic::MissionType::MISSION){

                }else if(component->getMissionType() == MissionTopic::MissionType::GUIDED){

                }else if(component->getMissionType() == MissionTopic::MissionType::ACTION){

                }
            }
        }
    }
}



