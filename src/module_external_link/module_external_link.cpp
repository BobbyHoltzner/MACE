#include "module_external_link.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////
///             CONFIGURE
////////////////////////////////////////////////////////////////////////////////////////////////////////

ModuleExternalLink::ModuleExternalLink() :
    m_VehicleDataTopic("vehicleData"),m_MissionDataTopic("vehicleMission"), associatedSystemID(255)
{
    MissionItem::MissionList missionList;
    missionList.setVehicleID(1);
    missionList.initializeQueue(4);

    std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> newWP = std::make_shared<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>();
    newWP->position.setPosition(35.7470021,-78.8395026,0.0);
    newWP->setVehicleID(1);

    std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> newWP1 = std::make_shared<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>();
    newWP1->position.setPosition(35.7463033,-78.8386631,0.0);
    newWP1->setVehicleID(1);

    std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> newWP2 = std::make_shared<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>();
    newWP2->position.setPosition(35.7459724,-78.8390923,0.0);
    newWP2->setVehicleID(1);

    std::shared_ptr<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>> newWP3 = std::make_shared<MissionItem::SpatialWaypoint<DataState::StateGlobalPosition>>();
    newWP3->position.setPosition(35.7466538,-78.8399184,0.0);
    newWP3->setVehicleID(1);


    MissionItem::MissionList newMissionList;
    newMissionList.insertMissionItem(newWP);
    newMissionList.insertMissionItem(newWP1);
    newMissionList.insertMissionItem(newWP2);
    //newMissionList.insertMissionItem(newWP3);

    m_VehicleCurrentMissionMap.insert({1,newMissionList});
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
                std::shared_ptr<MissionTopic::MissionHomeTopic> component = std::make_shared<MissionTopic::MissionHomeTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);
            }
        }
    }

}

///////////////////////////////////////////////////////////////////////////////////////
/// The following are public virtual functions imposed from IModuleCommandExternalLink.
///////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////
/// GENERAL VEHICLE COMMAND EVENTS: These are events that may have a direct
/// command and action sequence that accompanies the vheicle. Expect an
/// acknowledgement or an event to take place when calling these items.
////////////////////////////////////////////////////////////////////////////

void ModuleExternalLink::ChangeVehicleArm(const MissionItem::ActionArm &vehicleArm)
{
    mavlink_message_t msg = DataCOMMS::Command_MACETOCOMMS::generateArmMessage(vehicleArm,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleExternalLink::ChangeVehicleOperationalMode(const MissionItem::ActionChangeMode &vehicleMode)
{
    UNUSED(vehicleMode);
}

void ModuleExternalLink::RequestVehicleTakeoff(const MissionItem::SpatialTakeoff<DataState::StateGlobalPosition> &vehicleTakeoff)
{
    mavlink_message_t msg = DataCOMMS::Command_MACETOCOMMS::generateTakeoffMessage(vehicleTakeoff,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

/////////////////////////////////////////////////////////////////////////////
/// GENERAL HOME EVENTS: These events are related to establishing or setting
/// a home position. It should be recognized that the first mission item in a
/// mission queue should prepend this position. Just the way ardupilot works.
/////////////////////////////////////////////////////////////////////////////

void ModuleExternalLink::RequestVehicleHomePosition(const int &vehicleID)
{
    mavlink_message_t msg = DataCOMMS::Command_MACETOCOMMS::generateGetHomeMessage(vehicleID,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleExternalLink::SetVehicleHomePosition(const MissionItem::SpatialHome &vehicleHome)
{
    mavlink_message_t msg = DataCOMMS::Command_MACETOCOMMS::generateSetHomePosition(vehicleHome,m_LinkChan);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

/////////////////////////////////////////////////////////////////////////
/// GENERAL MISSION EVENTS: This is implying for auto mode of the vehicle.
/// This functionality may be pertinent for vehicles not containing a
/// direct MACE hardware module.
/////////////////////////////////////////////////////////////////////////

void ModuleExternalLink::SetCurrentMissionQueue(const MissionItem::MissionList &missionList)
{
    UNUSED(missionList);
}

void ModuleExternalLink::RequestCurrentMissionQueue(const int &vehicleID)
{
    mavlink_message_t msg;
    mavlink_mission_request_list_t list;
    list.target_system = vehicleID;
    list.target_component = 0;
    mavlink_msg_mission_request_list_encode_chan(associatedSystemID,0,m_LinkChan,&msg,&list);
    m_LinkMarshaler->SendMessage<mavlink_message_t>(m_LinkName, msg);
}

void ModuleExternalLink::RequestClearMissionQueue(const int &vehicleID)
{
    UNUSED(vehicleID);
}

////////////////////////////////////////////////////////////////////////////
/// GENERAL GUIDED EVENTS: This is implying for guided mode of the vehicle.
/// This functionality is pertinent for vehicles that may contain a
/// MACE HW module, or, vehicles that have timely or ever updating changes.
////////////////////////////////////////////////////////////////////////////

void ModuleExternalLink::SetCurrentGuidedQueue(const MissionItem::MissionList &missionList)
{
    UNUSED(missionList);
}

void ModuleExternalLink::RequestCurrentGuidedQueue(const int &vehicleID)
{
    UNUSED(vehicleID);
}

void ModuleExternalLink::RequestClearGuidedQueue(const int &vehicleID)
{
    UNUSED(vehicleID);
}


