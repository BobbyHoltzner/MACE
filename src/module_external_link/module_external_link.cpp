#include "module_external_link.h"


////////////////////////////////////////////////////////////////////////////////////////////////////////
///             CONFIGURE
////////////////////////////////////////////////////////////////////////////////////////////////////////

ModuleExternalLink::ModuleExternalLink() :
    m_VehicleDataTopic("vehicleData"),m_SensorFootprintDataTopic("sensorFootprint"),m_MissionDataTopic("vehicleMission")
{

}

//!
//! \brief This module as been attached as a module
//! \param ptr pointer to object that attached this instance to itself
//!
void ModuleExternalLink::AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)

{
//    ptr->Subscribe(this, m_VehicleDataTopic.Name());
//    ptr->Subscribe(this, m_SensorFootprintDataTopic.Name());
//    ptr->Subscribe(this, m_MissionDataTopic.Name());
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
    std::cout<<"A new mavlink message was recieved via external module and I like it."<<message.msgid<<std::endl;
//    //get maping of all vehicle data components
//    std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> components = m_MAVLINKParser.Parse(&message);

//    //proceed to send components only if there is 1 or more
//    if(components.size() > 0)
//    {
//        //construct datagram
//        MaceCore::TopicDatagram topicDatagram;
//        for(size_t i = 0 ; i < components.size() ; i++)
//        {
//            ModuleVehicleMavlinkBase::m_VehicleDataTopic.SetComponent(components.at(i), topicDatagram);
//        }

//        //notify listneres of topic
//        ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
//            ptr->NewTopicDataValues(ModuleVehicleMavlinkBase::m_VehicleDataTopic.Name(), 1, MaceCore::TIME(), topicDatagram);
//        });
//    }
}

void ModuleExternalLink::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{
    //In relevance to the external link module, the module when receiving a new topic should pack that up for transmission
    //to other instances of MACE

}

void ModuleExternalLink::NewlyAvailableVehicle(const int &vehicleID)
{

}


