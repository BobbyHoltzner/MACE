#define MAVLINK_NEED_BYTE_SWAP
#include <iostream>

#include "module_vehicle_mavlink.h"

#include "mace_core/module_factory.h"

#include <QSerialPort>

#include "comms/serial_link.h"
#include "comms/protocol_mavlink.h"

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

/*
void ModuleVehicleMAVLINK::gotInfoTest(const Data::VehicleStateData &messageData)
{
    int vID = 1;
    VFRData* tmpData = new VFRData(vID);
    tmpData->airspeedData = 10.0;

    //std::shared_ptr<AbstractVehicleMessage> tmpAbstractType = std::shared_ptr<AbstractVehicleMessage>(tmpData);
    VehicleMessage tmpMessage;
    tmpMessage.setDataObject(std::shared_ptr<AbstractVehicleMessage>(tmpData));
\
    switch(messageData.getVehicleData()->getProtocolDefinition())
    {
        case Data::PROTOCOL_ARDUPILOT:
            Data::ArducopterData *arducopterMessage = (Data::ArducopterData*)messageData.getVehicleData().get();
            gotArducopterMessage(*arducopterMessage);
            break;
    }
}
void ModuleVehicleMAVLINK::gotArducopterMessage(const Data::ArducopterData &messageArducopter)
{
    switch(messageArducopter.getMessageDef())
    {
        case Data::MESSAGE_ATTITUDE:
            Data::ArducopterAttitude arducopterAttitude = (Data::ArducopterAttitude&)messageArducopter;
        break;
    }
}
*/

ModuleVehicleMAVLINK::ModuleVehicleMAVLINK() :
    MaceCore::IModuleCommandVehicle(),
    m_LinkMarshler(new Comms::CommsMarshaler)
{
    m_LinkMarshler->AddSubscriber(this);
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
    protocolSettings->AddTerminalParameters("Name", MaceCore::ModuleParameterTerminalTypes::STRING, true, "Mavlink", {"Mavlink"});
    protocolSettings->AddTerminalParameters("Version", MaceCore::ModuleParameterTerminalTypes::STRING, true, "V1", {"V1", "V2"});

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

            m_LinkMarshler->AddProtocol(*mavlinkConfig);

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

        m_LinkMarshler->AddLink("link1", config);


        //now configure to use link with desired protocol
        if(protocolToUse == Comms::Protocols::MAVLINK)
        {
            m_LinkMarshler->SetProtocolForLink("link1", Comms::Protocols::MAVLINK);

            std::shared_ptr<Comms::MavlinkConfiguration> mavlinkConfig = std::static_pointer_cast<Comms::MavlinkConfiguration>(m_AvailableProtocols.at(Comms::Protocols::MAVLINK));

            //set version on mavlink channel
            // I would prefer to put this in Comms library, but because the mavlinkstatus is static variable, things get messed up when linking
            uint8_t chan = m_LinkMarshler->GetProtocolChannel("link1");
            mavlink_status_t* mavlinkStatus = mavlink_get_channel_status(chan);
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
        m_LinkMarshler->ConnectToLink("link1");


        //test statements that will issue a log_request_list to device
        uint8_t chan = m_LinkMarshler->GetProtocolChannel("link1");
        mavlink_message_t msg;
        mavlink_msg_log_request_list_pack_chan(255,190, chan,&msg,0,0,0,0xFFFF);
        m_LinkMarshler->SendMessage<mavlink_message_t>("link1", msg);

        //mavlink_msg_request_data_stream_pack_chan(255,190,chan,&msg,0,0,10,4,1);
        //m_LinkMarshler->SendMessage<mavlink_message_t>("link1", msg);

        //param 1 is the message id
        //interval between two messages in microseconds

        //mavlink_msg_command_long_pack_chan(255,190,chan,&msg,0,0,511,0,30,500000,0,0,0,0,0);
        //m_LinkMarshler->SendMessage<mavlink_message_t>("link1", msg);


//        mavlink_msg_request_data_stream_pack_chan(255,190,chan,&msg,0,0,11,4,1);
//        m_LinkMarshler->SendMessage<mavlink_message_t>("link1", msg);

//        mavlink_msg_request_data_stream_pack_chan(255,190,chan,&msg,0,0,12,4,1);
//        m_LinkMarshler->SendMessage<mavlink_message_t>("link1", msg);


    }
    else
    {
        throw std::runtime_error("No Link has been configured for the vehicle MAVLINK module");
    }

}


////////////////////////////////////////////////////////////////////////////////////////////////////////
///              MACE COMMANDS
////////////////////////////////////////////////////////////////////////////////////////////////////////


void ModuleVehicleMAVLINK::CreateVehicleObject(const int &vehicleID)
{
    std::list<int>::iterator it;
    for (it=m_NeededVehicleObjects.begin(); it != m_NeededVehicleObjects.end(); ++it)
    {
        if(*it == vehicleID)
        {
            //std::cout<<"This item was already in the list to update when more info is learned."<<std::endl;
            //This implies that the module is already aware an object needs to be created
            break;
        }
    }
    if(it == m_NeededVehicleObjects.end()){
        //std::cout<<"This item was not already in the list adding one with the ID of: "<<vehicleID<<std::endl;
        m_NeededVehicleObjects.push_back(vehicleID);
    }
}

void ModuleVehicleMAVLINK::RemoveVehicleObject(const int &sendersID)
{
    //std::cout<<"I have been told to remove the vehicle object from my list with the ID of: "<<sendersID<<std::endl;
    for (auto it=m_NeededVehicleObjects.begin(); it != m_NeededVehicleObjects.end(); ++it)
    {
        if(*it == sendersID)
        {
            //std::cout<<"I am removing it"<<std::endl;
            it = m_NeededVehicleObjects.erase(it);
            break;
        }
    }
}

//!
//! \brief New commands have been updated that the vehicle is to follow immediatly
//!
//! Commands are to be retreived through the MaceData available through getDataObject()
//!
void ModuleVehicleMAVLINK::FollowNewCommands()
{

}


//!
//! \brief New commands have been issued to vehicle that are to be followed once current command is finished
//!
//! Commands are to be retreived through the MaceData available through getDataObject()
//!
void ModuleVehicleMAVLINK::FinishAndFollowNewCommands()
{

}


//!
//! \brief New commands have been appended to existing commands.
//!
//! Commands are to be retreived through the MaceData available through getDataObject()
//!
void ModuleVehicleMAVLINK::CommandsAppended()
{

}


////////////////////////////////////////////////////////////////////////////////////////////////////////
///              COMM EVENTS
////////////////////////////////////////////////////////////////////////////////////////////////////////
void ModuleVehicleMAVLINK::vehicleObjectCheck(const int &sendersID, const int &autopilotType) const
{
    std::cout<<"The senders ID I am looking for is: "<<sendersID<<std::endl;
    for (auto it=m_NeededVehicleObjects.begin(); it != m_NeededVehicleObjects.end(); ++it)
    {
        std::cout<<"The ID at this position in the map is: "<<*it<<std::endl;

        if(*it == sendersID)
        {
            switch (autopilotType) {
            case MAV_AUTOPILOT_ARDUPILOTMEGA:
            {
                Ardupilot::DataArdupilot tmpObject(sendersID,1,1);
                //std::cout<<"The value at this iterator position matches the sending ID!"<<std::endl;
                std::shared_ptr<Ardupilot::DataArdupilot> tmpArdupilot = std::make_shared<Ardupilot::DataArdupilot>(tmpObject);

                NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
                       ptr->NewConstructedVehicle(this,tmpArdupilot);
                    });
                break;
            }
            default:
                std::cout<<"The type of autopilot seen with ID: "<<sendersID<<" is not currently supported."<<std::endl;
                break;
            }

            break;
        }
    }
}

//!
//! \brief A Message has been received over Mavlink protocol
//! \param linkName Link identifier which generated call
//! \param message Message that has been received
//!
void ModuleVehicleMAVLINK::MavlinkMessage(const std::string &linkName, const mavlink_message_t &message) const
{
    int sendersID = (int)message.sysid;
    int messageID = (int)message.msgid;

    if(sendersID == 51)
    {
        return;
    }
    std::cout<<"The senders ID seen here is: "<<sendersID<<std::endl;
    GenericMsgDef_MAVLINK<mavlink_message_t>* tmpMsgObj = new GenericMsgDef_MAVLINK<mavlink_message_t>(sendersID, message);

    if(messageID == MAVLINK_MSG_ID_HEARTBEAT)
    {
        mavlink_heartbeat_t decodedMSG;
        mavlink_msg_heartbeat_decode(&message,&decodedMSG);
        std::cout<<"The custom mode is: "<<decodedMSG.custom_mode<<std::endl;

        if(m_NeededVehicleObjects.size() != 0)
        {
            //std::cout<<"The size of the list is currently: "<<m_NeededVehicleObjects.size()<<std::endl;
            vehicleObjectCheck(sendersID,(int)decodedMSG.autopilot);
        }
    }

    VehicleMessage tmpMessage;
    tmpMessage.setDataObject(std::shared_ptr<AbstractVehicleMessage>(tmpMsgObj));
    NotifyListeners([&](MaceCore::IModuleEventsVehicle* ptr){
        ptr->NewVehicleMessage(this,MaceCore::TIME(),tmpMessage);
    });

}


//!
//! \brief New heartbeat from MAVLINK received over a link
//! \param linkName Name of link
//! \param vehicleId
//! \param vehicleMavlinkVersion
//! \param vehicleFirmwareType
//! \param vehicleType
//!
void ModuleVehicleMAVLINK::VehicleHeartbeatInfo(const std::string &linkName, int vehicleId, int vehicleMavlinkVersion, int vehicleFirmwareType, int vehicleType) const
{
    //incomming heartbeats
}

