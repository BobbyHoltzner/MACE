#include "module_external_link.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////
///             CONFIGURE
////////////////////////////////////////////////////////////////////////////////////////////////////////

ModuleExternalLink::ModuleExternalLink() :
    m_VehicleDataTopic("vehicleData"),m_MissionDataTopic("vehicleMission"), associatedSystemID(255),firstHearbeat(true)
{

}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleExternalLink::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
{
    ptr->Subscribe(this, m_VehicleDataTopic.Name());
    ptr->Subscribe(this, m_MissionDataTopic.Name());
}

//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleExternalLink::ModuleConfigurationStructure() const
{

    MaceCore::ModuleParameterStructure structure;
    ConfigureMAVLINKStructure(structure);
    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleExternalLink::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    ConfigureComms(params);
}

//!
//! \brief New Mavlink message received over a link
//! \param linkName Name of link message received over
//! \param msg Message received
//!
void ModuleExternalLink::MavlinkMessage(const std::string &linkName, const mavlink_message_t &message)
{
    UNUSED(linkName);
    this->ParseForData(&message);
}

void ModuleExternalLink::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    //In relevance to the external link module, the module when receiving a new topic should pack that up for transmission
    //to other instances of MACE
    //example read of vehicle data
    if(topicName == m_VehicleDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), senderID);

        //example of how to get data and parse through the components that were updated
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == DataStateTopic::StateAttitudeTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateAttitudeTopic> component = std::make_shared<DataStateTopic::StateAttitudeTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                mavlink_message_t msg = DataCOMMS::State_MACETOCOMMS::AttitudeTopicPTR_MACETOCOMMS(component,senderID,0,m_LinkChan);
                m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> component = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                mavlink_message_t msg = DataCOMMS::State_MACETOCOMMS::GlobalPositionTopicPTR_MACETOCOMMS(component,senderID,0,m_LinkChan);
                m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
            }
            else if(componentsUpdated.at(i) == DataGenericItemTopic::DataGenericItemTopic_FlightMode::Name()) {
                std::shared_ptr<DataGenericItemTopic::DataGenericItemTopic_FlightMode> component = std::make_shared<DataGenericItemTopic::DataGenericItemTopic_FlightMode>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                mavlink_message_t msg = DataCOMMS::Generic_MACETOCOMMS::FlightModeTopicPTR_MACETOCOMMS(component,senderID,0,m_LinkChan);
                associatedSystemID = senderID;
                m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
            }
        }
    }
    else if(topicName == m_MissionDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_MissionDataTopic.Name(), senderID);

        //example of how to get data and parse through the components that were updated
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == MissionTopic::MissionListTopic::Name()) {
                std::shared_ptr<MissionTopic::MissionListTopic> component = std::make_shared<MissionTopic::MissionListTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);
            }
            else if(componentsUpdated.at(i) == MissionTopic::MissionHomeTopic::Name()) {
                //We have received an updated HomePositionTopic
                std::shared_ptr<MissionTopic::MissionHomeTopic> component = std::make_shared<MissionTopic::MissionHomeTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);
                MissionItem::SpatialHome newHome = component->getHome();

                DataCOMMS::Mission_MACETOCOMMS commsTranslator(newHome.getVehicleID(),m_LinkChan);
                mavlink_message_t msg = commsTranslator.Home_MACETOCOMMS(newHome);
                m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
            }
        }
    }

}

///////////////////////////////////////////////////////////////////////////////////////
/// The following are public virtual functions imposed from IModuleCommandExternalLink.
//////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
/// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
/// command and action sequence that accompanies the vheicle. Expect an
/// acknowledgement or an event to take place when calling these items.
////////////////////////////////////////////////////////////////////////////

void ModuleExternalLink::Command_ChangeVehicleArm(const MissionItem::ActionArm &vehicleArm)
{
    mavlink_message_t msg = DataCOMMS::Command_MACETOCOMMS::generateArmMessage(vehicleArm,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleExternalLink::Command_ChangeVehicleOperationalMode(const MissionItem::ActionChangeMode &vehicleMode)
{
    UNUSED(vehicleMode);
}

void ModuleExternalLink::Command_RequestVehicleTakeoff(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &vehicleTakeoff)
{
    mavlink_message_t msg = DataCOMMS::Command_MACETOCOMMS::generateTakeoffMessage(vehicleTakeoff,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

/////////////////////////////////////////////////////////////////////////////
/// GENERAL HOME EVENTS: These events are related to establishing or setting
/// a home position. It should be recognized that the first mission item in a
/// mission queue should prepend this position. Just the way ardupilot works.
/////////////////////////////////////////////////////////////////////////////

void ModuleExternalLink::Command_GetHomePosition(const int &vehicleID)
{
    mavlink_message_t msg = DataCOMMS::Command_MACETOCOMMS::generateGetHomeMessage(vehicleID,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleExternalLink::Command_SetHomePosition(const MissionItem::SpatialHome &vehicleHome)
{
    mavlink_message_t msg = DataCOMMS::Command_MACETOCOMMS::generateSetHomePosition(vehicleHome,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

/////////////////////////////////////////////////////////////////////////
/// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
/// This functionality may be pertinent for vehicles not containing a
/// direct MACE hardware module.
/////////////////////////////////////////////////////////////////////////

void ModuleExternalLink::Command_UploadMission(const MissionItem::MissionList &missionList)
{
    MissionItem::MissionList::MissionListStatus status = missionList.getMissionListStatus();

    if(status.state == MissionItem::MissionList::COMPLETE)
    {
        mavlink_mace_new_proposed_mission_t missionProposed;
        missionProposed.count = missionList.getQueueSize();
        Data::MissionKey key = missionList.getMissionKey();
        missionProposed.mission_creator = key.m_creatorID;
        missionProposed.mission_id = key.m_missionID;
        missionProposed.mission_state = static_cast<MACE_MISSION_STATE>(missionList.getMissionTypeState());
        missionProposed.mission_type = static_cast<MACE_MISSION_TYPE>(key.m_missionType);
        missionProposed.target_system = key.m_systemID;

        mavlink_message_t msg;
        mavlink_msg_mace_new_proposed_mission_encode_chan(associatedSystemID,0,m_LinkChan,&msg,&missionProposed);
        m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);

    }
}

void ModuleExternalLink::Command_GetMission(const Data::MissionKey &key)
{
    UNUSED(key);
}

void ModuleExternalLink::Command_SetCurrentMission(const Data::MissionKey &key)
{
    UNUSED(key);
}

void ModuleExternalLink::Command_GetCurrentMission(const int &targetSystem)
{
    UNUSED(targetSystem);
}

void ModuleExternalLink::Command_ClearCurrentMission(const int &targetSystem)
{
    UNUSED(targetSystem);
}

void ModuleExternalLink::Command_GetOnboardAuto(const int &targetSystem)
{
    mavlink_message_t msg;
    mavlink_mace_mission_request_list_t request;
    request.target_system = targetSystem;
    request.mission_type = MACE_MISSION_TYPE_AUTO;
    request.mission_state = MACE_MISSION_ONBOARD;
    mavlink_msg_mace_mission_request_list_encode_chan(associatedSystemID,0,m_LinkChan,&msg,&request);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleExternalLink::Command_ClearOnboardAuto(const int &targetSystem)
{
    UNUSED(targetSystem);
}

void ModuleExternalLink::Command_GetOnboardGuided(const int &targetSystem)
{
    mavlink_message_t msg;
    mavlink_mace_mission_request_list_t request;
    request.target_system = targetSystem;
    request.mission_type = MACE_MISSION_TYPE_GUIDED;
    request.mission_state = MACE_MISSION_ONBOARD;
    mavlink_msg_mace_mission_request_list_encode_chan(associatedSystemID,0,m_LinkChan,&msg,&request);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleExternalLink::Command_ClearOnboardGuided(const int &targetSystem)
{
    UNUSED(targetSystem);
}




