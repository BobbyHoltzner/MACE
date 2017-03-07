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
    ptr->Subscribe(this, m_VehicleDataTopic.Name());
    ptr->Subscribe(this, m_SensorFootprintDataTopic.Name());
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
    //example read of vehicle data
    if(topicName == m_VehicleDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), senderID);

        //example of how to get data and parse through the components that were updated
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
//            std::cout << "  " << componentsUpdated.at(i) << std::endl;
            if(componentsUpdated.at(i) == DataStateTopic::StateAttitudeTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateAttitudeTopic> component = std::make_shared<DataStateTopic::StateAttitudeTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);

                // Write Attitude data to the GUI:
                sendAttitudeData(senderID, component);
            }
            else if(componentsUpdated.at(i) == DataArdupilot::VehicleFlightMode::Name()) {
                std::shared_ptr<DataArdupilot::VehicleFlightMode> component = std::make_shared<DataArdupilot::VehicleFlightMode>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                std::cout << "    Vehicle Type: " << (int)component->getVehicleType() << std::endl;
                std::cout << "    Vehicle Mode: " << (int)component->getFlightMode() << std::endl;
            }
            else if(componentsUpdated.at(i) == DataArdupilot::VehicleOperatingStatus::Name()) {
                std::shared_ptr<DataArdupilot::VehicleOperatingStatus> component = std::make_shared<DataArdupilot::VehicleOperatingStatus>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                std::cout << "    Vehicle Armed: " << component->getVehicleArmed() << std::endl;
            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> component = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
                std::cout << "    lat: " << component->latitude << " long: " << component->longitude << std::endl;

                // Write Position data to the GUI:
                sendPositionData(senderID, component);
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

                // Write mission items to the GUI:
                sendVehicleMission(senderID, component);
            }
            else if(componentsUpdated.at(i) == MissionTopic::MissionHomeTopic::Name()) {
                std::shared_ptr<MissionTopic::MissionHomeTopic> component = std::make_shared<MissionTopic::MissionHomeTopic>();
                m_MissionDataTopic.GetComponent(component, read_topicDatagram);

                // Write mission items to the GUI:
                sendVehicleHome(senderID, component);
            }
        }
    }

}

void ModuleExternalLink::NewlyAvailableVehicle(const int &vehicleID)
{

}


