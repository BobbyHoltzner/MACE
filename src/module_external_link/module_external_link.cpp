#include "module_external_link.h"


////////////////////////////////////////////////////////////////////////////////////////////////////////
///             CONFIGURE
////////////////////////////////////////////////////////////////////////////////////////////////////////

ModuleExternalLink::ModuleExternalLink() :
    m_VehicleDataTopic("vehicleData"),m_SensorFootprintDataTopic("sensorFootprint"),m_MissionDataTopic("vehicleMission"),
    m_LinkMarshaler(new Comms::CommsMarshaler), m_LinkName(""), m_LinkChan(0)
{
    m_LinkMarshaler->AddSubscriber(this);

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
    std::shared_ptr<MaceCore::ModuleParameterStructure> serialSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    serialSettings->AddTerminalParameters("PortName", MaceCore::ModuleParameterTerminalTypes::STRING, true);
    serialSettings->AddTerminalParameters("BaudRate", MaceCore::ModuleParameterTerminalTypes::INT, true);
    serialSettings->AddTerminalParameters("DataBits", MaceCore::ModuleParameterTerminalTypes::INT, true);
    serialSettings->AddTerminalParameters("StopBits", MaceCore::ModuleParameterTerminalTypes::INT, true);
    serialSettings->AddTerminalParameters("Parity", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, true);
    serialSettings->AddTerminalParameters("FlowControl", MaceCore::ModuleParameterTerminalTypes::INT, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> protocolSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    protocolSettings->AddTerminalParameters("Name", MaceCore::ModuleParameterTerminalTypes::STRING, true, "Mavlink", {"Mavlink"});
    protocolSettings->AddTerminalParameters("Version", MaceCore::ModuleParameterTerminalTypes::STRING, true, "V1", {"V1", "V2"});

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleExternalLink::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    std::shared_ptr<Comms::ProtocolConfiguration> protocolConfig;
    if(params->HasNonTerminal("ProtocolParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> protocolSettings = params->GetNonTerminalValue("ProtocolParameters");
        std::string protocolName = protocolSettings->GetTerminalValue<std::string>("Name");
        std::string versionName = protocolSettings->GetTerminalValue<std::string>("Version");


        if(protocolName == "Mavlink")
        {
            std::shared_ptr<Comms::MavlinkConfiguration> mavlinkConfig = std::make_shared<Comms::MavlinkConfiguration>();

            if(versionName == "V1")
            {
                mavlinkConfig->SetVersion(Comms::MavlinkConfiguration::MavlinkVersion::MavlinkVersionAlways1);
            }
            else if(versionName == "V2")
            {
                mavlinkConfig->SetVersion(Comms::MavlinkConfiguration::MavlinkVersion::MavlinkVersionAlways2);
            }
            else
            {
                throw std::runtime_error("Unknown mavlink version seen");
            }

            m_LinkMarshaler->AddProtocol(*mavlinkConfig);

            m_AvailableProtocols.insert({Comms::Protocols::MAVLINK, std::static_pointer_cast<Comms::ProtocolConfiguration>(mavlinkConfig)});
            protocolConfig = mavlinkConfig;
        }
        else
        {
            throw std::runtime_error("Unknown Protocol encountered");
        }

    }
    if(params->HasNonTerminal("SerialParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> serialSettings = params->GetNonTerminalValue("SerialParameters");


        std::string portName = serialSettings->GetTerminalValue<std::string>("PortName");
        int baudRate = serialSettings->GetTerminalValue<int>("BaudRate");
        int dataBits = serialSettings->GetTerminalValue<int>("DataBits");
        int stopBits = serialSettings->GetTerminalValue<int>("StopBits");

        bool parity = serialSettings->GetTerminalValue<bool>("Parity");
        int flowControl = serialSettings->GetTerminalValue<int>("FlowControl");


        Comms::Protocols protocolToUse = Comms::Protocols::MAVLINK;

        Comms::SerialConfiguration config("config");
        config.setPortName(portName);
        config.setBaud(baudRate);
        config.setDataBits(dataBits);
        config.setStopBits(stopBits);
        config.setParity(parity);
        config.setFlowControl(flowControl);

        m_LinkName = "link1";
        m_LinkMarshaler->AddLink(m_LinkName, config);


        //now configure to use link with desired protocol
        if(protocolToUse == Comms::Protocols::MAVLINK)
        {
            m_LinkMarshaler->SetProtocolForLink(m_LinkName, Comms::Protocols::MAVLINK);

            std::shared_ptr<Comms::MavlinkConfiguration> mavlinkConfig = std::static_pointer_cast<Comms::MavlinkConfiguration>(m_AvailableProtocols.at(Comms::Protocols::MAVLINK));

            //set version on mavlink channel
            // I would prefer to put this in Comms library, but because the mavlinkstatus is static variable, things get messed up when linking
            m_LinkChan = m_LinkMarshaler->GetProtocolChannel(m_LinkName);
            mavlink_status_t* mavlinkStatus = mavlink_get_channel_status(m_LinkChan);
            std::cout << mavlinkStatus << std::endl;
            switch (mavlinkConfig->GetVersion()) {
            case Comms::MavlinkConfiguration::MavlinkVersion::MavlinkVersion2IfVehicle2:
                if (mavlinkStatus->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) {
                    mavlinkStatus->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
                    break;
                }
                // Fallthrough to set version 2
            case Comms::MavlinkConfiguration::MavlinkVersion::MavlinkVersionAlways2:
                mavlinkStatus->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
                break;
            default:
            case Comms::MavlinkConfiguration::MavlinkVersion::MavlinkVersionAlways1:
                mavlinkStatus->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
                break;
            }
        }
        //connect link
        bool success = m_LinkMarshaler->ConnectToLink(m_LinkName);
        if(success == false) {
            throw std::runtime_error("Connection to link failed");
        }
    }
    else
    {
        throw std::runtime_error("No Link has been configured for the external comms module");
    }
}

void ModuleExternalLink::NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated)
{

    count++;

    if(count > 50 && executedOnce == false)
    {
        executedOnce = true;
//        MissionItem::ActionChangeMode newVehicleMode;
//        newVehicleMode.setRequestMode("STABILIZE");
//        newVehicleMode.setVehicleID(senderID);

        ModuleExternalLink::NotifyListeners([&](MaceCore::IModuleEventsGeneral* ptr){
            ptr->RequestCurrentVehicleMission(this, senderID);
        });
    }
    //example read of vehicle data
    if(topicName == m_VehicleDataTopic.Name())
    {
        //get latest datagram from mace_data
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_VehicleDataTopic.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == DataStateTopic::StateAttitudeTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateAttitudeTopic> component = std::make_shared<DataStateTopic::StateAttitudeTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
            }
            else if(componentsUpdated.at(i) == DataStateTopic::StateGlobalPositionTopic::Name()) {
                std::shared_ptr<DataStateTopic::StateGlobalPositionTopic> component = std::make_shared<DataStateTopic::StateGlobalPositionTopic>();
                m_VehicleDataTopic.GetComponent(component, read_topicDatagram);
            }
        }
    }else if(topicName == m_SensorFootprintDataTopic.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_SensorFootprintDataTopic.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == DataVehicleSensors::SensorVertices_Local::Name()) {
                std::shared_ptr<DataVehicleSensors::SensorVertices_Local> newSensorV = std::make_shared<DataVehicleSensors::SensorVertices_Local>("TestM");
                m_SensorFootprintDataTopic.GetComponent(newSensorV, read_topicDatagram);
            }
            if(componentsUpdated.at(i) == DataVehicleSensors::SensorVertices_Global::Name()) {
                std::shared_ptr<DataVehicleSensors::SensorVertices_Global> newSensorV = std::make_shared<DataVehicleSensors::SensorVertices_Global>("TestM");
                m_SensorFootprintDataTopic.GetComponent(newSensorV, read_topicDatagram);
            }
        }
    }
    else if(topicName == m_MissionDataTopic.Name())
    {
        MaceCore::TopicDatagram read_topicDatagram = this->getDataObject()->GetCurrentTopicDatagram(m_MissionDataTopic.Name(), senderID);
        for(size_t i = 0 ; i < componentsUpdated.size() ; i++) {
            if(componentsUpdated.at(i) == MissionTopic::MissionHomeTopic::Name()) {
                std::shared_ptr<MissionTopic::MissionHomeTopic> newHome = std::make_shared<MissionTopic::MissionHomeTopic>();
                m_MissionDataTopic.GetComponent(newHome, read_topicDatagram);
            }
            else if(componentsUpdated.at(i) == MissionTopic::MissionListTopic::Name()) {
                //I should do something when I see a new mission from the vehicle
            }
        }
    }
}

void ModuleExternalLink::NewlyAvailableVehicle(const int &vehicleID)
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////////
///              COMM EVENTS
////////////////////////////////////////////////////////////////////////////////////////////////////////


//!
//! \brief New Mavlink message received over a link
//! \param linkName Name of link message received over
//! \param msg Message received
//!
virtual void ModuleExternalLink::MavlinkMessage(const std::string &linkName, const mavlink_message_t &message)
{
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

