#define MAVLINK_NEED_BYTE_SWAP

#include "module_vehicle_mavlink.h"

#include "mace_core/module_factory.h"

#include <QSerialPort>

#include "comms/serial_link.h"
#include "comms/mavlink_protocol.h"


/*
 *
 * EXAMPLE ON HOW TO GENERATE MAVLINK MESSAGE:
 *
 *   uint8_t chan = m_LinkMarshler->GetProtocolChannel(link);
 *   mavlink_message_t msg;
 *   mavlink_msg_log_request_list_pack_chan(255,190, chan,&msg,0,0,0,0xFFFF);
 *   m_LinkMarshler->SendMessage<mavlink_message_t>(link, msg);
 *
 * */


////////////////////////////////////////////////////////////////////////////////////////////////////////
///             CONFIGURE
////////////////////////////////////////////////////////////////////////////////////////////////////////


ModuleVehicleMAVLINK::ModuleVehicleMAVLINK() :
    MaceCore::IModuleCommandVehicle(),
    m_LinkMarshler(new Comms::LinkMarshaler)
{
}


//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleVehicleMAVLINK::ModuleConfigurationStructure() const
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
    protocolSettings->AddTerminalParameters("Name", MaceCore::ModuleParameterTerminalTypes::STRING, true);
    protocolSettings->AddTerminalParameters("Version", MaceCore::ModuleParameterTerminalTypes::STRING, true);

    structure.AddNonTerminal("SerialParameters", serialSettings, true);
    structure.AddNonTerminal("ProtocolParameters", protocolSettings, true);

    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModuleVehicleMAVLINK::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{
    if(params->HasNonTerminal("ProtocolParameters"))
    {
        std::shared_ptr<MaceCore::ModuleParameterValue> protocolSettings = params->GetNonTerminalValue("ProtocolParameters");
        std::string protocolName = protocolSettings->GetTerminalValue<std::string>("Name");
        std::string versionName = protocolSettings->GetTerminalValue<std::string>("Version");

        if(protocolName == "Mavlink")
        {
            std::shared_ptr<Comms::MavlinkComms> protocol = std::make_shared<Comms::MavlinkComms>();
            protocol->AddListner(this);

            if(versionName == "V1")
            {
                protocol->SetVersion(Comms::MavlinkComms::MavlinkVersion::MavlinkVersionAlways1);
            }

            m_LinkMarshler->AddProtocol(Comms::Protocols::MAVLINK, std::dynamic_pointer_cast<Comms::IProtocol>(protocol));
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



        Comms::SerialConfiguration config("config");
        config.setPortName(portName);
        config.setBaud(baudRate);
        config.setDataBits(dataBits);
        config.setStopBits(stopBits);
        config.setParity(parity);
        config.setFlowControl(flowControl);

        std::shared_ptr<Comms::SerialLink> link = std::make_shared<Comms::SerialLink>(config);
        m_LinkMarshler->AddLink(link);

        m_LinkMarshler->SetProtocolForLink(link, Comms::Protocols::MAVLINK);

        link->Connect();

        //test statements that will issue a log_request_list to device
        //uint8_t chan = m_LinkMarshler->GetProtocolChannel(link);
        //mavlink_message_t msg;
        //mavlink_msg_log_request_list_pack_chan(255,190, chan,&msg,0,0,0,0xFFFF);
        //m_LinkMarshler->SendMessage<mavlink_message_t>(link, msg);
    }
    else
    {
        throw std::runtime_error("No Link has been configured for the vehicle MAVLINK module");
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
///              MACE COMMANDS
////////////////////////////////////////////////////////////////////////////////////////////////////////


//!
//! \brief New commands have been updated that the vehicle is to follow immediatly
//!
//! Commands are to be retreived through the MaceData available through getDataObject()
//! Method will be called on module's thread
//!
void ModuleVehicleMAVLINK::FollowNewCommands()
{

}


//!
//! \brief New commands have been issued to vehicle that are to be followed once current command is finished
//!
//! Commands are to be retreived through the MaceData available through getDataObject()
//! Method will be called on module's thread
//!
void ModuleVehicleMAVLINK::FinishAndFollowNewCommands()
{

}


//!
//! \brief New commands have been appended to existing commands.
//!
//! Commands are to be retreived through the MaceData available through getDataObject()
//! Method will be called on module's thread
//!
void ModuleVehicleMAVLINK::CommandsAppended()
{

}


////////////////////////////////////////////////////////////////////////////////////////////////////////
///              PROTOCOL EVENTS
////////////////////////////////////////////////////////////////////////////////////////////////////////

//!
//! \brief A message about protocol has been generated
//! \param title
//! \param message
//!
void ModuleVehicleMAVLINK::ProtocolStatusMessage(const std::string &title, const std::string &message) const
{
}


//!
//! \brief A Message has been received over Mavlink protocol
//! \param message Message that has been received
//!
void ModuleVehicleMAVLINK::MessageReceived(const mavlink_message_t &message) const
{
    //std::cout << "Message" << std::endl;
    std::cout << "  ID: " << message.msgid << std::endl;
    if(message.msgid == 118)
        std::cout << "AsdfaSD" << std::endl;
}


//!
//! \brief Heartbeat of vehicle received
//! \param link
//! \param vehicleId
//! \param vehicleMavlinkVersion
//! \param vehicleFirmwareType
//! \param vehicleType
//!
void ModuleVehicleMAVLINK::VehicleHeartbeatInfo(const Comms::ILink* link, int vehicleId, int vehicleMavlinkVersion, int vehicleFirmwareType, int vehicleType) const
{
    //std::cout << "Heartbeat" << std::endl;
}


void ModuleVehicleMAVLINK::ReceiveLossPercentChanged(int uasId, float lossPercent) const
{

}


void ModuleVehicleMAVLINK::ReceiveLossTotalChanged(int uasId, int totalLoss) const
{

}


//!
//! \brief A new radio status packet received
//! \param link
//! \param rxerrors
//! \param fixed
//! \param rssi
//! \param remrssi
//! \param txbuf
//! \param noise
//! \param remnoise
//!
void ModuleVehicleMAVLINK::RadioStatusChanged(const Comms::ILink *link, unsigned rxerrors, unsigned fixed, int rssi, int remrssi, unsigned txbuf, unsigned noise, unsigned remnoise) const
{

}
