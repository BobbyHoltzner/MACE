#ifndef MODULE_EXTERNAL_LINK_H
#define MODULE_EXTERNAL_LINK_H

#include "module_external_link_global.h"

#include "mace_core/i_module_topic_events.h"
#include "mace_core/i_module_command_external_link.h"
#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data_vehicle_sensors/components.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"

//__________________________________________________________________
#include <iostream>
#include <QThread>
#include <QSerialPort>

#include "common/common.h"

#include "comms/comms_marshaler.h"
#include "comms/i_protocol_mavlink_events.h"
#include "comms/serial_configuration.h"

#include "comms/serial_link.h"
#include "comms/udp_link.h"
#include "comms/protocol_mavlink.h"

#include "data_vehicle_MAVLINK/mavlink_parser.h"

#include "mace_core/module_factory.h"


class MODULE_EXTERNAL_LINKSHARED_EXPORT ModuleExternalLink :
        public MaceCore::IModuleCommandExternalLink,
        public Comms::CommsEvents
{

public:
    ModuleExternalLink();
    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    virtual void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr);

    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const;


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params);

    virtual void NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated);


    //! Virtual functions as defined by IModuleExternalLink
public:

    virtual void NewlyAvailableVehicle(const int &vehicleID);


private:
    bool executedOnce = false;
    int count = 0;
    Data::TopicDataObjectCollection<DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;
    Data::TopicDataObjectCollection<DATA_MISSION_GENERIC_TOPICS> m_MissionDataTopic;

    //Data::TopicDataObjectCollection<DATA_VEHICLE_ARDUPILOT_TYPES, DATA_VEHICLE_MAVLINK_TYPES, DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;
    Data::TopicDataObjectCollection<DATA_VEHICLE_SENSOR_FOOTPRINT> m_SensorFootprintDataTopic;

};




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

        std::shared_ptr<MaceCore::ModuleParameterStructure> protocolSettings = std::make_shared<MaceCore::ModuleParameterStructure>();
        protocolSettings->AddTerminalParameters("Name", MaceCore::ModuleParameterTerminalTypes::STRING, true, "Mavlink", {"Mavlink"});
        protocolSettings->AddTerminalParameters("Version", MaceCore::ModuleParameterTerminalTypes::STRING, true, "V1", {"V1", "V2"});

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
        else
        {
            throw std::runtime_error("No Link has been configured for the external comms module");
        }

    }

    virtual std::unordered_map<std::string, MaceCore::TopicStructure> GetTopics()
    {
        return {};
    }

public:
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
protected:
    Comms::CommsMarshaler *m_LinkMarshaler;
    std::string m_LinkName;
    uint8_t m_LinkChan;

private:
    std::unordered_map<Comms::Protocols, std::shared_ptr<Comms::ProtocolConfiguration>, EnumClassHash> m_AvailableProtocols;

    DataVehicleMAVLINK::MAVLINKParser m_MAVLINKParser;

};

#endif // MODULE_EXTERNAL_LINK_H
