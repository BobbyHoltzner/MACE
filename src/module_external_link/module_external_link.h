#ifndef MODULE_EXTERNAL_LINK_H
#define MODULE_EXTERNAL_LINK_H

#include "module_external_link_global.h"

#include <mavlink.h>

#include "commsMAVLINK/comms_mavlink.h"

#include "mace_core/i_module_topic_events.h"
#include "mace_core/i_module_command_external_link.h"
#include "data/i_topic_component_data_object.h"
#include "data/topic_data_object_collection.h"

#include "data_vehicle_sensors/components.h"
#include "data_vehicle_MAVLINK/components.h"
#include "data_vehicle_ardupilot/components.h"

#include "data_vehicle_ardupilot/components/vehicle_operating_status.h"

#include "data_generic_state_item/state_item_components.h"
#include "data_generic_state_item_topic/state_topic_components.h"

#include "data_generic_mission_item/mission_item_components.h"
#include "data_generic_mission_item_topic/mission_item_topic_components.h"


class MODULE_EXTERNAL_LINKSHARED_EXPORT ModuleExternalLink :
        public MaceCore::IModuleCommandExternalLink,
        public CommsMAVLINK
{

public:

    ModuleExternalLink();

    //!
    //! \brief New Mavlink message received over a link
    //! \param linkName Name of link message received over
    //! \param msg Message received
    //!
    virtual void MavlinkMessage(const std::string &linkName, const mavlink_message_t &msg);

    //!
    //! \brief NewTopic
    //! \param topicName
    //! \param senderID
    //! \param componentsUpdated
    //!
    virtual void NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated);

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



public:
    //! Virtual functions as defined by IModuleExternalLink

    virtual void NewlyAvailableVehicle(const int &vehicleID);

private:
    Data::TopicDataObjectCollection<DATA_VEHICLE_ARDUPILOT_TYPES, DATA_VEHICLE_MAVLINK_TYPES, DATA_STATE_GENERIC_TOPICS> m_VehicleDataTopic;
    Data::TopicDataObjectCollection<DATA_MISSION_GENERIC_TOPICS> m_MissionDataTopic;

};

#endif // MODULE_EXTERNAL_LINK_H
