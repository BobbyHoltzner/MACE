#ifndef MODULE_VEHICLE_MAVLINK_H
#define MODULE_VEHICLE_MAVLINK_H

#define MAVLINK_NEED_BYTE_SWAP

#include "module_vehicle_mavlink_global.h"

#include <iostream>
#include <QMap>
#include <QThread>
#include <QSerialPort>

#include "common/common.h"

#include "module_vehicle_generic/module_vehicle_generic.h"

#include "comms/comms_marshaler.h"
#include "comms/i_protocol_mavlink_events.h"
#include "comms/serial_configuration.h"

#include "comms/serial_link.h"
#include "comms/udp_link.h"
#include "comms/protocol_mavlink.h"

#include "Vehicles/Ardupilot/data_ardupilot.h"
#include "data_vehicle_MAVLINK/mavlink_parser.h"

#include "module_vehicle_mavlink.h"

#include "mace_core/module_factory.h"

#include "data_vehicle_MAVLINK/altitude_reference_frames.h"

#include "data_vehicle_MAVLINK/components.h"

/*
 *
 * USAGE:
 *
 * Insert the nessessary code to do Vehicle communications with MAVLINK
 *
 * Look at i_module_events_vehicle.h in mace_core for the events that can be triggered.
 * Feel free to add any nessessary events (if an event is added, its handler must also be added in mace_core.h
 *
 * When it comes time to signal an event to MaceCore do so by calling the following code structure:
 *      NotifyListeners([&](IModuleEventsVehicle *obj){obj->NewPositionDynamics(this, arg1, arg2, ... , argN);});
 * Replacing "NewPositionDynamics" with the event of your choice, and replacing arguments with what is required for that event
 *
 * The start method is the entry point for the thread that the module is to run on.
 * The start() method should contain an event loop of some sort that responds to commands made.
 *
 * Each module will implement commands as defined by it's interface.
 * These commands will NOT be invoked on the thread the module is operating on.
 * If the command is to kick off some action on the module's thread, it will have to marshaled onto the event loop in some way.
 *
 * */

template <typename ...VehicleTopicAdditionalComponents>
class MODULE_VEHICLE_MAVLINKSHARED_EXPORT ModuleVehicleMAVLINK :
        public ModuleVehicleGeneric<VehicleTopicAdditionalComponents..., DATA_VEHICLE_MAVLINK_TYPES>,
        public Comms::CommsEvents
{
protected:
    typedef ModuleVehicleGeneric<VehicleTopicAdditionalComponents..., DATA_VEHICLE_MAVLINK_TYPES> ModuleVehicleMavlinkBase;

public:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///             CONFIGURE
    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ModuleVehicleMAVLINK() :
        ModuleVehicleGeneric<VehicleTopicAdditionalComponents..., DataVehicleMAVLINK::GPSStatus>(),
        m_LinkMarshaler(new Comms::CommsMarshaler), m_LinkName(""), m_LinkChan(0)
    {
        m_LinkMarshaler->AddSubscriber(this);
    }


    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const
    {
        MaceCore::ModuleParameterStructure structure;
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

        return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
    }


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
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

            //  TODO-PAT: Everything above this to the previous "TODO-PAT" should be moved onto a thread
            // ********************************************************************************************

            //connect link
            if(m_LinkMarshaler->ConnectToLink(m_LinkName) == false){
                throw std::runtime_error("Connection to udp link failed");
            }



    //        //test statements that will issue a set_mode to device (ID=1)
    //        uint8_t chan = m_LinkMarshaler->GetProtocolChannel(linkName);
    //        mavlink_message_t msg;
    //        mavlink_msg_set_mode_pack(255, 190, &msg, 1, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0);
    //        m_LinkMarshaler->SendMessage<mavlink_message_t>(linkName, msg);
        }
        else
        {
            throw std::runtime_error("No Link has been configured for the vehicle MAVLINK module");
        }

    }

    virtual std::unordered_map<std::string, MaceCore::TopicStructure> GetTopics()
    {
        //return IModuleCommandVehicle::GetTopics();
        return {};
    }

public:

    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///              MACE COMMANDS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    virtual void ChangeVehicleArm(const MissionItem::ActionArm &vehicleArm)
    {

    }

    virtual void ChangeVehicleOperationalMode(const MissionItem::ActionChangeMode &vehicleMode)
    {

    }

    virtual void CreateVehicleObject(const int &vehicleID)
    {
        std::list<int>::iterator it;
        for (it=m_NeededVehicleObjects.begin(); it != m_NeededVehicleObjects.end(); ++it)
        {
            if(*it == vehicleID)
            {
                //This implies that the module is already aware an object needs to be created
                break;
            }
        }
        if(it == m_NeededVehicleObjects.end()){
            m_NeededVehicleObjects.push_back(vehicleID);
        }
    }

    virtual void RemoveVehicleObject(const int &vehicleID)
    {
        m_NeededVehicleObjects.remove(vehicleID);
    }

    virtual void UpdateVehicleObjectList(const std::list<int> &vehicleObjectList)
    {
        m_NeededVehicleObjects = vehicleObjectList;
    }


    //!
    //! \brief New commands have been updated that the vehicle is to follow immediatly
    //!
    //! Commands are to be retreived through the MaceData available through getDataObject()
    //!
    //!
    virtual void FollowNewCommands()
    {

    }


    //!
    //! \brief New commands have been issued to vehicle that are to be followed once current command is finished
    //!
    //! Commands are to be retreived through the MaceData available through getDataObject()
    //!
    virtual void FinishAndFollowNewCommands()
    {

    }


    //!
    //! \brief New commands have been appended to existing commands
    //!
    //! Commands are to be retreived through the MaceData available through getDataObject()
    //!
    virtual void CommandsAppended()
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
    virtual void MavlinkMessage(const std::string &linkName, const mavlink_message_t &message)
    {
        //get maping of all vehicle data components
        std::vector<std::shared_ptr<Data::ITopicComponentDataObject>> components = m_MAVLINKParser.Parse(&message);

        //proceed to send components only if there is 1 or more
        if(components.size() > 0)
        {
            //construct datagram
            MaceCore::TopicDatagram topicDatagram;
            for(size_t i = 0 ; i < components.size() ; i++)
            {
                ModuleVehicleMavlinkBase::m_VehicleDataTopic.SetComponent(components.at(i), topicDatagram);
            }

            //notify listneres of topic
            ModuleVehicleMavlinkBase::NotifyListenersOfTopic([&](MaceCore::IModuleTopicEvents* ptr){
                ptr->NewTopicDataValues(ModuleVehicleMavlinkBase::m_VehicleDataTopic.Name(), 1, MaceCore::TIME(), topicDatagram);
            });
        }
    }


    //!
    //! \brief New heartbeat from MAVLINK received over a link
    //! \param linkName Name of link
    //! \param vehicleId
    //! \param vehicleMavlinkVersion
    //! \param vehicleFirmwareType
    //! \param vehicleType
    //!
    virtual void VehicleHeartbeatInfo(const std::string &linkName, int vehicleId, int vehicleMavlinkVersion, int vehicleFirmwareType, int vehicleType) const
    {
        //incomming heartbeats
    }
protected:
    Comms::CommsMarshaler *m_LinkMarshaler;
    std::string m_LinkName;
    uint8_t m_LinkChan;

private:

    mutable std::list<int> m_NeededVehicleObjects;

    std::unordered_map<Comms::Protocols, std::shared_ptr<Comms::ProtocolConfiguration>, EnumClassHash> m_AvailableProtocols;

    DataVehicleMAVLINK::MAVLINKParser m_MAVLINKParser;

};

#endif // MODULE_VEHICLE_MAVLINK_H
