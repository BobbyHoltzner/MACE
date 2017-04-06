#include "module_vehicle_ardupilot.h"

ModuleVehicleArdupilot::ModuleVehicleArdupilot() :
    ModuleVehicleMAVLINK<DATA_VEHICLE_ARDUPILOT_TYPES>(),
    m_VehicleMission("vehicleMission"),m_CurrentMissionItem(NULL)
{

}

DataARDUPILOT::VehicleObject_ARDUPILOT* ModuleVehicleArdupilot::getArducopterData(const int &systemID)
{
    DataARDUPILOT::VehicleObject_ARDUPILOT* tmpData;
    try{
        tmpData = m_ArduPilotData.at(systemID);
    }catch(const std::out_of_range &oor)
    {
        tmpData = new DataARDUPILOT::VehicleObject_ARDUPILOT(systemID,255,0);
        m_ArduPilotData.insert({systemID,tmpData});
        //Initialize the appropriate mission/guided queues in the data sets
        tmpData->data->m_CurrentGuidedQueue.setVehicleID(systemID);
        tmpData->data->m_ProposedGuidedQueue.setVehicleID(systemID);
        tmpData->data->m_CurrentMissionQueue.setVehicleID(systemID);
        tmpData->data->m_ProposedMissionQueue.setVehicleID(systemID);

        ModuleVehicleMavlinkBase::NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
            ptr->NewConstructedVehicle(this, systemID);
        });
    }

    return tmpData;
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
    int vehicleID = vehicleArm.getVehicleID();
    DataARDUPILOT::VehicleObject_ARDUPILOT* tmpData = getArducopterData(vehicleID);
    mavlink_message_t msg = tmpData->generateArmMessage(vehicleArm,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::ChangeVehicleOperationalMode(const MissionItem::ActionChangeMode &vehicleMode)
{
    int vehicleID = vehicleMode.getVehicleID();
    std::string modeString = vehicleMode.getRequestMode();

    DataARDUPILOT::VehicleObject_ARDUPILOT* tmpData = getArducopterData(vehicleID);
    if(tmpData->data->heartbeatSeen)
    {
        int newFlightMode = tmpData->data->m_ArducopterFlightMode->getFlightModeFromString(modeString);
        mavlink_message_t msg = tmpData->generateChangeMode(vehicleID,m_LinkChan,newFlightMode);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
    }
}

void ModuleVehicleArdupilot::RequestVehicleTakeoff(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &vehicleTakeoff)
{
    int vehicleID = vehicleTakeoff.getVehicleID();
    DataARDUPILOT::VehicleObject_ARDUPILOT* tmpData = getArducopterData(vehicleID);
    mavlink_message_t msg = tmpData->generateTakeoffMessage(vehicleTakeoff,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

/////////////////////////////////////////////////////////////////////////////
/// GENERAL HOME EVENTS: These events are related to establishing or setting
/// a home position. It should be recognized that the first mission item in a
/// mission queue should prepend this position. Just the way ardupilot works.
/////////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::RequestVehicleHomePosition(const int &vehicleID)
{
    DataARDUPILOT::VehicleObject_ARDUPILOT* tmpData = getArducopterData(vehicleID);
    mavlink_message_t msg = tmpData->generateGetHomeMessage(vehicleID,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::SetVehicleHomePosition(const MissionItem::SpatialHome &vehicleHome)
{
    int vehicleID = vehicleHome.getVehicleID();
    DataARDUPILOT::VehicleObject_ARDUPILOT* tmpData = getArducopterData(vehicleID);
    mavlink_message_t msg = tmpData->generateSetHomePosition(vehicleHome,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

/////////////////////////////////////////////////////////////////////////
/// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
/// This functionality may be pertinent for vehicles not containing a
/// direct MACE hardware module.
/////////////////////////////////////////////////////////////////////////

void ModuleVehicleArdupilot::SetMissionQueue(const MissionItem::MissionList &missionList)
{
    int vehicleID = missionList.getVehicleID();
    DataARDUPILOT::VehicleObject_ARDUPILOT* tmpData = getArducopterData(vehicleID);
    tmpData->data->m_ProposedMissionQueue = missionList;

    //m_ProposedMissionQueue[vehicleID] = missionList;
    mavlink_message_t msg;
    int queueSize = tmpData->data->m_ProposedMissionQueue.getQueueSize();
    mavlink_msg_mission_count_pack_chan(255,190,m_LinkChan,&msg,vehicleID,0,queueSize);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::GetMissionQueue(const Data::SystemDescription &targetSystem)
{
    mavlink_message_t msg;
    mavlink_msg_mission_request_list_pack_chan(255,190,m_LinkChan,&msg,targetSystem.getSystemID(),0);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleVehicleArdupilot::ClearMissionQueue(const Data::SystemDescription &targetSystem)
{
    //This is message number 45....
    //TODO: Do we get an acknowledgement from this?
    mavlink_message_t msg;
    mavlink_msg_mission_clear_all_pack_chan(255,190,m_LinkChan,&msg,targetSystem.getSystemID(),0);
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
    DataARDUPILOT::VehicleObject_ARDUPILOT* tmpData = getArducopterData(vehicleID);
    UNUSED(tmpData);
}

void ModuleVehicleArdupilot::RequestCurrentGuidedQueue(const int &vehicleID)
{
    //This command is performed locally in the MACE instance.
    DataARDUPILOT::VehicleObject_ARDUPILOT* tmpData = getArducopterData(vehicleID);
    UNUSED(tmpData);
}

void ModuleVehicleArdupilot::RequestClearGuidedQueue(const int &vehicleID)
{
    DataARDUPILOT::VehicleObject_ARDUPILOT* tmpData = getArducopterData(vehicleID);
    tmpData->data->m_CurrentGuidedQueue.clearQueue();
    UNUSED(tmpData);
}


void ModuleVehicleArdupilot::VehicleHeartbeatInfo(const std::string &linkName, const int systemID, const mavlink_heartbeat_t &heartbeatMSG)
{
    UNUSED(linkName);
    DataARDUPILOT::VehicleObject_ARDUPILOT* tmpData = getArducopterData(systemID);
    //The purpose of this module seeing if it is the first heartbeat is to establish initial comms and parameter grabs
    //from the vehicle.
    if(!tmpData->data->heartbeatSeen)
    {
        tmpData->data->heartbeatSeen = true;
        mavlink_message_t msg;
        mavlink_msg_mission_request_list_pack_chan(255,190,m_LinkChan,&msg,systemID,0);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);

        Ardupilot_GuidedController newController;
        m_ArdupilotController[systemID] = newController;

        std::thread thread([&newController]()
        {
            newController.runGuidanceRoutine();
        });
    }

    tmpData->data->m_ArducopterFlightMode->parseMAVLINK(heartbeatMSG);

    MaceCore::TopicDatagram topicDatagram;
    m_VehicleDataTopic.SetComponent(tmpData->data->m_ArducopterFlightMode, topicDatagram);
    //notify listneres of topic
    ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
        ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), systemID, MaceCore::TIME(), topicDatagram);
    });
}

//!
//! \brief New Mavlink message received over a link
//! \param linkName Name of link message received over
//! \param msg Message received
//!
void ModuleVehicleArdupilot::MavlinkMessage(const std::string &linkName, const mavlink_message_t &message)
{
    int systemID = message.sysid;
    DataARDUPILOT::VehicleObject_ARDUPILOT* tmpData = getArducopterData(systemID);

    bool wasMissionMSG = ParseMAVLINKMissionMessage(tmpData, linkName, &message);

    if(wasMissionMSG == false){
        //generate topic datagram from given mavlink message
        std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> components = tmpData->parser->ParseForVehicleData(&message);
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



