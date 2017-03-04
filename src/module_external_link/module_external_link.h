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

    virtual std::unordered_map<std::string, MaceCore::TopicStructure> GetTopics()
    {
        return {};
    }

    virtual void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr);

    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const;

    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params);

    virtual void NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated);


    ////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///              COMM EVENTS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

    virtual void MavlinkMessage(const std::string &linkName, const mavlink_message_t &message);

    virtual void VehicleHeartbeatInfo(const std::string &linkName, int vehicleId, int vehicleMavlinkVersion, int vehicleFirmwareType, int vehicleType) const
    {
        //incomming heartbeats
    }

public:
    //! Virtual functions as defined by IModuleExternalLink

    virtual void NewlyAvailableVehicle(const int &vehicleID);


//________________________________________
protected:
    Comms::CommsMarshaler *m_LinkMarshaler;
    std::string m_LinkName;
    uint8_t m_LinkChan;

private:
    std::unordered_map<Comms::Protocols, std::shared_ptr<Comms::ProtocolConfiguration>, EnumClassHash> m_AvailableProtocols;
    DataVehicleMAVLINK::MAVLINKParser m_MAVLINKParser;
//________________________________________

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


};

#endif // MODULE_EXTERNAL_LINK_H
