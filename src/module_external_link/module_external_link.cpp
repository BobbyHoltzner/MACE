#include "module_external_link.h"


////////////////////////////////////////////////////////////////////////////////////////////////////////
///             CONFIGURE
////////////////////////////////////////////////////////////////////////////////////////////////////////

ModuleExternalLink::ModuleExternalLink() :
    m_VehicleDataTopic("vehicleData"),m_MissionDataTopic("vehicleMission")
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
    //This function will be receiving messages external to the specific MACE instance that is deploying this

    switch ((int)message.msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        //might want to figure out a way to handle the case of sending an
        //empty heartbeat back just to acknowledge the aircraft is still there
        //then again the streaming other messages may handle this...so maybe
        //timer should be since last time heard.
        mavlink_heartbeat_t decodedMSG;
        mavlink_msg_heartbeat_decode(&message,&decodedMSG);

        std::shared_ptr<DataArdupilot::VehicleFlightMode> ptrParameters = std::make_shared<DataArdupilot::VehicleFlightMode>();
        ptrParameters->parseMAVLINK(decodedMSG);
        ptrParameters->setVehicleArmed(decodedMSG.base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY);

        MaceCore::TopicDatagram topicDatagram;
        m_VehicleDataTopic.SetComponent(ptrParameters, topicDatagram);

        ModuleExternalLink::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
            ptr->NewTopicDataValues(this, m_VehicleDataTopic.Name(), message.sysid, MaceCore::TIME(), topicDatagram);
        });
        break;
    }
    }
}

void ModuleExternalLink::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    std::cout<<"I just saw a new topic with name of: "<<topicName<<std::endl;
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
            }
            else if(componentsUpdated.at(i) == DataArdupilot::VehicleFlightMode::Name()) {
                std::shared_ptr<DataArdupilot::VehicleFlightMode> component = std::make_shared<DataArdupilot::VehicleFlightMode>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> component = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
            }
        }
    }
    else if(topicName == m_MissionDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_MissionDataTopic.Name(), senderID);

        //example of how to get data and parse through the components that were updated
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
//            std::cout << "  " << componentsUpdated.at(i) << std::endl;
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

void ModuleExternalLink::NewlyAvailableVehicle(const int &vehicleID)
{
    UNUSED(vehicleID);
}


