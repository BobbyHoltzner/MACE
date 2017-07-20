#include "comms_mavlink.h"
CommsMAVLINK::CommsMAVLINK() :
    m_LinkMarshaler(new Comms::CommsMarshaler), m_LinkName(""), m_LinkChan(0)
{
    m_LinkMarshaler->AddSubscriber(this);
}

CommsMAVLINK::~CommsMAVLINK()
{

}

void CommsMAVLINK::MavlinkHeartbeat(const std::string &linkName, const int &systemID, const mavlink_heartbeat_t &heartbeatMSG)
{
    UNUSED(linkName);
    UNUSED(heartbeatMSG);
}

void CommsMAVLINK::MavlinkMessage(const std::string &linkName, const mavlink_message_t &message)
{
    UNUSED(linkName);
    UNUSED(message);
    std::cout<<"I am in the comms_mavlink library MavlinkMessage callback."<<std::endl;
}

//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
void CommsMAVLINK::ConfigureMAVLINKStructure(MaceCore::ModuleParameterStructure &structure) const
{
    std::shared_ptr<MaceCore::ModuleParameterStructure> serialSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    serialSettings->AddTerminalParameters("PortName", MaceCore::ModuleParameterTerminalTypes::STRING, true);
    serialSettings->AddTerminalParameters("BaudRate", MaceCore::ModuleParameterTerminalTypes::INT, true);
    serialSettings->AddTerminalParameters("DataBits", MaceCore::ModuleParameterTerminalTypes::INT, true);
    serialSettings->AddTerminalParameters("StopBits", MaceCore::ModuleParameterTerminalTypes::INT, true);
    serialSettings->AddTerminalParameters("Parity", MaceCore::ModuleParameterTerminalTypes::BOOLEAN, true);
    serialSettings->AddTerminalParameters("FlowControl", MaceCore::ModuleParameterTerminalTypes::INT, true);
    //        structure.AddNonTerminal("SerialParameters", serialSettings, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> udpSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    udpSettings->AddTerminalParameters("ListenAddress", MaceCore::ModuleParameterTerminalTypes::STRING, true);
    udpSettings->AddTerminalParameters("ListenPortNumber", MaceCore::ModuleParameterTerminalTypes::INT, true);

    std::shared_ptr<MaceCore::ModuleParameterStructure> protocolSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
    protocolSettings->AddTerminalParameters("Name", MaceCore::ModuleParameterTerminalTypes::STRING, true, "Mavlink", {"Mavlink"});
    protocolSettings->AddTerminalParameters("Version", MaceCore::ModuleParameterTerminalTypes::STRING, true, "V1", {"V1", "V2"});

    structure.AddMutuallyExclusiveNonTerminal({{"SerialParameters", serialSettings}, {"UDPParameters", udpSettings}}, false);
    structure.AddNonTerminal("ProtocolParameters", protocolSettings, true);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void CommsMAVLINK::ConfigureComms(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
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

        m_LinkName = "link_" + portName;
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
        }

        bool success = m_LinkMarshaler->ConnectToLink(m_LinkName);
        if(success == false) {
            throw std::runtime_error("Connection to link failed");
        }
    }
    else if(params->HasNonTerminal("UDPParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> udpSettings = params->GetNonTerminalValue("UDPParameters");

        std::string listenAddress = udpSettings->GetTerminalValue<std::string>("ListenAddress");
        int listenPortNumber = udpSettings->GetTerminalValue<int>("ListenPortNumber");

        Comms::Protocols protocolToUse = Comms::Protocols::MAVLINK;
        Comms::UdpConfiguration config(listenAddress, listenPortNumber);

        // ********************************************************************************************
        // TODO-PAT: This function is blocking while it listens for the sender port.
        //             --Need to figure out a way to move this to a thread to execute in the background until
        //                  a UDP connection is seen on this address and port number.
//            config.listenForPort(listenPortNumber);

        m_LinkName = "udplink_" + std::to_string(listenPortNumber);
        m_LinkMarshaler->AddUDPLink(m_LinkName, config);

        //now configure to use link with desired protocol
        if(protocolToUse == Comms::Protocols::MAVLINK)
        {
            m_LinkMarshaler->SetProtocolForLink(m_LinkName, Comms::Protocols::MAVLINK);

            std::shared_ptr<Comms::MavlinkConfiguration> mavlinkConfig = std::static_pointer_cast<Comms::MavlinkConfiguration>(m_AvailableProtocols.at(Comms::Protocols::MAVLINK));

            //set version on mavlink channel
            // I would prefer to put this in Comms library, but because the mavlinkstatus is static variable, things get messed up when linking
            m_LinkChan = m_LinkMarshaler->GetProtocolChannel(m_LinkName);
            mavlink_status_t* mavlinkStatus = mavlink_get_channel_status(m_LinkChan);
        }

        //  TODO-PAT: Everything above this to the previous "TODO-PAT" should be moved onto a thread
        // ********************************************************************************************

        //connect link
        if(m_LinkMarshaler->ConnectToLink(m_LinkName) == false){
            throw std::runtime_error("Connection to udp link failed");
        }
    }
    else
    {
        throw std::runtime_error("No Link has been configured for the vehicle MAVLINK module");
    }

}
