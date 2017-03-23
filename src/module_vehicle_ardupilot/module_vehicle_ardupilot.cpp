#include "module_vehicle_ardupilot.h"

ModuleVehicleArdupilot::ModuleVehicleArdupilot() :
    ModuleVehicleMAVLINK<DATA_VEHICLE_ARDUPILOT_TYPES>(),
    m_VehicleMission("vehicleMission"),m_CurrentMissionItem(NULL)
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

////////////////////////////////////////////////////////////////////////////
/// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
/// command and action sequence that accompanies the vheicle. Expect an
/// acknowledgement or an event to take place when calling these items.
////////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::ChangeVehicleArm(const MissionItem::ActionArm &vehicleArm)
{
    DataMAVLINK::Command_MACETOMAVLINK commandObject;
    mavlink_message_t msg = commandObject.generateArmMessage(vehicleArm,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::ChangeVehicleOperationalMode(const MissionItem::ActionChangeMode &vehicleMode)
{
    int vehicleID = vehicleMode.getVehicleID();
    std::string modeString = vehicleMode.getRequestMode();

    DataArdupilot::DataVehicleArdupilot* tmpData = m_ArduPilotData.at(vehicleID);
    if(tmpData->heartbeatUpdated())
    {
        int newFlightMode = tmpData->getFlightModeFromString(modeString);
        mavlink_message_t msg = DataArdupilot::generateChangeMode(vehicleID,m_LinkChan,newFlightMode);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
    }
}

void ModuleVehicleArdupilot::RequestVehicleTakeoff(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &vehicleTakeoff)
{
    int vehicleID = vehicleTakeoff.getVehicleID();
    DataArdupilot::DataVehicleArdupilot* tmpData = m_ArduPilotData.at(vehicleID);
    UNUSED(tmpData);
    DataMAVLINK::Command_MACETOMAVLINK commandObject;
    mavlink_message_t msg = commandObject.generateTakeoffMessage(vehicleTakeoff,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

/////////////////////////////////////////////////////////////////////////////
/// GENERAL HOME EVENTS: These events are related to establishing or setting
/// a home position. It should be recognized that the first mission item in a
/// mission queue should prepend this position. Just the way ardupilot works.
/////////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::RequestVehicleHomePosition(const int &vehicleID)
{
    DataMAVLINK::Command_MACETOMAVLINK commandObject;
    mavlink_message_t msg = commandObject.generateGetHomeMessage(vehicleID,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::SetVehicleHomePosition(const MissionItem::SpatialHome &vehicleHome)
{
    mavlink_message_t msg = DataArdupilot::generateSetHomePosition(vehicleHome,m_LinkChan);
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
    DataArdupilot::DataVehicleArdupilot* tmpData = m_ArduPilotData.at(vehicleID);
    tmpData->m_ProposedMissionQueue = missionList;

    //m_ProposedMissionQueue[vehicleID] = missionList;
    mavlink_message_t msg;
    int queueSize = tmpData->m_ProposedMissionQueue.getQueueSize();
    mavlink_msg_mission_count_pack_chan(255,190,m_LinkChan,&msg,vehicleID,0,queueSize);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);

//    delete tmpData;
//    tmpData = NULL;
}

void ModuleVehicleArdupilot::RequestCurrentMissionQueue(const int &vehicleID)
{
    mavlink_message_t msg;
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
    DataArdupilot::DataVehicleArdupilot* tmpData = m_ArduPilotData.at(vehicleID);
    UNUSED(tmpData);
}

void ModuleVehicleArdupilot::RequestCurrentGuidedQueue(const int &vehicleID)
{
    //This command is performed locally in the MACE instance.
    DataArdupilot::DataVehicleArdupilot* tmpData = m_ArduPilotData.at(vehicleID);
    UNUSED(tmpData);

//    delete tmpData;
//    tmpData = NULL;
}

void ModuleVehicleArdupilot::RequestClearGuidedQueue(const int &vehicleID)
{
    DataArdupilot::DataVehicleArdupilot* tmpData = m_ArduPilotData.at(vehicleID);
    tmpData->m_CurrentGuidedQueue.clearQueue();
//    delete tmpData;
//    tmpData = NULL;
}


//!
//! \brief New Mavlink message received over a link
//! \param linkName Name of link message received over
//! \param msg Message received
//!
void ModuleVehicleArdupilot::MavlinkMessage(const std::string &linkName, const mavlink_message_t &message)
{
    ModuleVehicleMAVLINK<DATA_VEHICLE_ARDUPILOT_TYPES>::MavlinkMessage(linkName, message);

    DataArdupilot::DataVehicleArdupilot* tmpData;
    int systemID = message.sysid;

    try{
        tmpData = m_ArduPilotData.at(systemID);
    }catch(const std::out_of_range &oor)
    {
        tmpData = new DataArdupilot::DataVehicleArdupilot();
        m_ArduPilotData.insert({systemID,tmpData});

        //Initialize the appropriate mission/guided queues in the data sets
        tmpData->m_CurrentGuidedQueue.setVehicleID(systemID);
        tmpData->m_ProposedGuidedQueue.setVehicleID(systemID);
        tmpData->m_CurrentMissionQueue.setVehicleID(systemID);
        tmpData->m_ProposedMissionQueue.setVehicleID(systemID);


        MissionItem::MissionList newMissionList;
        m_CurrentMissionQueue.insert({systemID,newMissionList});
        m_ProposedMissionQueue.insert({systemID,newMissionList});

        m_CurrentGuidedQueue.insert({systemID,newMissionList});
        m_ProposedGuidedQueue.insert({systemID,newMissionList});

        ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
            ptr->NewConstructedVehicle(this, systemID);
        });
    }

    bool wasMissionMSG = ParseMAVLINKMissionMessage(linkName, &message);

    if(wasMissionMSG == false){
        //generate topic datagram from given mavlink message
        std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> components = tmpData->ParseForVehicleData(&message);
        //proceed to send components only if there is 1 or more
        if(components.size() > 0)
        {
            //construct datagram
            MaceCore::TopicDatagram topicDatagram;
            for(size_t i = 0 ; i < components.size() ; i++)
            {
                m_VehicleDataTopic.SetComponent(components.at(i), topicDatagram);
                //notify listneres of topic
                ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                    ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
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



